#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

class ImageFrame
{
    public:
    ImageFrame(){};
    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &_points,
               const vector<float> &_lidar_initialization_info,
               double _t) : t{_t}, is_key_frame{false}, reset_id{-1}, gravity{9.805}
    {
        points = _points;

        // reset id in case lidar odometry relocate
        reset_id = (int)round(_lidar_initialization_info[0]);
        // Pose
        T.x() = _lidar_initialization_info[1];
        T.y() = _lidar_initialization_info[2];
        T.z() = _lidar_initialization_info[3];
        // Rotation
        Eigen::Quaterniond Q = Eigen::Quaterniond(_lidar_initialization_info[7],
                                                  _lidar_initialization_info[4],
                                                  _lidar_initialization_info[5],
                                                  _lidar_initialization_info[6]);
        R = Q.normalized().toRotationMatrix();
        // Velocity
        V.x() = _lidar_initialization_info[8];
        V.y() = _lidar_initialization_info[9];
        V.z() = _lidar_initialization_info[10];
        // Acceleration bias
        Ba.x() = _lidar_initialization_info[11];
        Ba.y() = _lidar_initialization_info[12];
        Ba.z() = _lidar_initialization_info[13];
        // Gyroscope bias
        Bg.x() = _lidar_initialization_info[14];
        Bg.y() = _lidar_initialization_info[15];
        Bg.z() = _lidar_initialization_info[16];
        // Gravity
        gravity = _lidar_initialization_info[17];
    };

    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> points;
    double t;

    IntegrationBase *pre_integration;
    bool is_key_frame;

    // Lidar odometry info
    int reset_id;
    Vector3d T;
    Matrix3d R;
    Vector3d V;
    Vector3d Ba;
    Vector3d Bg;
    double gravity;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d *Bgs, Vector3d &g, VectorXd &x);

class odometryRegister
{
public:
    rclcpp::Node::SharedPtr node_;
    Eigen::Quaterniond vins_world_q_odom;
    tf2::Quaternion vins_world_tf_odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_latest_odometry;

    odometryRegister(rclcpp::Node::SharedPtr node) : node_(node)
    {
        // rotate position by pi, (w, x, y, z) // mark: from odom to vins_world
        vins_world_tf_odom.setRPY(0, 0, M_PI);
        vins_world_q_odom = Eigen::Quaterniond(0, 0, 0, 1);
        // pub_latest_odometry = node_->create_publisher<nav_msgs::msg::Odometry>("odometry/test", 1000);
    }

    // convert odometry from ROS Lidar frame to VINS camera frame

    // lidar_Rot_imu represents lidar^Rot_imu
    vector<float> getOdometry(deque<nav_msgs::msg::Odometry> &odomQueue, double img_time, Eigen::Matrix3d lidar_Rot_imu, Eigen::Vector3d lidar_Trans_imu)
    {
        vector<float> odometry_channel;
        odometry_channel.resize(18, -1); // reset id(1), P(3), Q(4), V(3), Ba(3), Bg(3), gravity(1)

        nav_msgs::msg::Odometry odomCur;

        // pop old odometry msg
        while (!odomQueue.empty())
        {
            if (rclcpp::Time(odomQueue.front().header.stamp).seconds() < img_time - 0.05)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
        {
            return odometry_channel;
        }

        // find the odometry time that is the closest to image time
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            odomCur = odomQueue[i];
            double imu_step = 1.0 / imu_Hz;
            if (rclcpp::Time(odomCur.header.stamp).seconds() < img_time - imu_step) // 500Hz imu 0.002 400Hz imu 0.0025 100hz 0.01
                continue;
            else
                break;
        }

        // time stamp difference still too large 原版 0.05
        if (abs(rclcpp::Time(odomCur.header.stamp).seconds() - img_time) > 0.05)
        {
            cout << "time stamp difference still too large" << endl;
            return odometry_channel;
        }
        //求解姿态
        // convert odometry rotation from lidar ROS frame to VINS camera frame (only rotation, assume lidar, camera, and IMU are close enough)
        tf2::Quaternion odom_R_lidar;
        tf2::fromMsg(odomCur.pose.pose.orientation, odom_R_lidar);
        /**
         * @brief modified
         *lidar_T_imu represents lidar^T_imu
         */
        tf2::Transform odom_T_lidar(odom_R_lidar, tf2::Vector3(odomCur.pose.pose.position.x, odomCur.pose.pose.position.y, odomCur.pose.pose.position.z));
        Eigen::Matrix3d extRot = lidar_Rot_imu; // lidar^Rot_imu
        Eigen::Vector3d extTrans(lidar_Trans_imu);
        Eigen::Vector3d ypr = extRot.eulerAngles(2, 1, 0);
        tf2::Quaternion lidar_q_imu;
        lidar_q_imu.setRPY(ypr.z(), ypr.y(), ypr.x());
        tf2::Transform lidar_T_imu(lidar_q_imu, tf2::Vector3(extTrans.x(), extTrans.y(), extTrans.z()));
        tf2::Transform odom_T_imu = odom_T_lidar * lidar_T_imu;
        odomCur.pose.pose.position.x = odom_T_imu.getOrigin().x();
        odomCur.pose.pose.position.y = odom_T_imu.getOrigin().y();
        odomCur.pose.pose.position.z = odom_T_imu.getOrigin().z();
        // Rotate orientation to VINS world
        tf2::Quaternion world_q_imu = vins_world_tf_odom * odom_T_imu.getRotation();
        odomCur.pose.pose.orientation = tf2::toMsg(world_q_imu);

        // convert odometry position from lidar ROS frame to VINS camera frame
        // 将位置求解
        Eigen::Vector3d p_eigen(odomCur.pose.pose.position.x, odomCur.pose.pose.position.y, odomCur.pose.pose.position.z);
        Eigen::Vector3d v_eigen(odomCur.twist.twist.linear.x, odomCur.twist.twist.linear.y, odomCur.twist.twist.linear.z);
        p_eigen = vins_world_q_odom * p_eigen;
        v_eigen = vins_world_q_odom * v_eigen;

        odomCur.pose.pose.position.x = p_eigen.x();
        odomCur.pose.pose.position.y = p_eigen.y();
        odomCur.pose.pose.position.z = p_eigen.z();
        // odomCur.pose.pose.position.z = 0;

        odomCur.twist.twist.linear.x = v_eigen.x();
        odomCur.twist.twist.linear.y = v_eigen.y();
        odomCur.twist.twist.linear.z = v_eigen.z();

        // odomCur.header.stamp = ros::Time().fromSec(img_time);

        // odomCur.header.frame_id = "vins_world";
        // odomCur.child_frame_id = "vins_body";
        // pub_latest_odometry.publish(odomCur);

        odometry_channel[0] = odomCur.pose.covariance[0];
        odometry_channel[1] = odomCur.pose.pose.position.x;
        odometry_channel[2] = odomCur.pose.pose.position.y;
        odometry_channel[3] = odomCur.pose.pose.position.z;
        odometry_channel[4] = odomCur.pose.pose.orientation.x;
        odometry_channel[5] = odomCur.pose.pose.orientation.y;
        odometry_channel[6] = odomCur.pose.pose.orientation.z;
        odometry_channel[7] = odomCur.pose.pose.orientation.w;
        odometry_channel[8] = odomCur.twist.twist.linear.x;
        odometry_channel[9] = odomCur.twist.twist.linear.y;
        odometry_channel[10] = odomCur.twist.twist.linear.z;
        odometry_channel[11] = odomCur.pose.covariance[1];
        odometry_channel[12] = odomCur.pose.covariance[2];
        odometry_channel[13] = odomCur.pose.covariance[3];
        odometry_channel[14] = odomCur.pose.covariance[4];
        odometry_channel[15] = odomCur.pose.covariance[5];
        odometry_channel[16] = odomCur.pose.covariance[6];
        odometry_channel[17] = odomCur.pose.covariance[7];

        return odometry_channel;
    }
};