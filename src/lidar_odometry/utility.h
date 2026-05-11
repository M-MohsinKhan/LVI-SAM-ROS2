#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include "mahonyMine.h"

using namespace std;

typedef pcl::PointXYZI PointType;

class ParamServer : public rclcpp::Node
{
public:
    Mahony filter;

    std::string PROJECT_NAME;

    std::string robot_id;

    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Velodyne Sensor Configuration: Velodyne
    int N_SCAN;
    int Horizon_SCAN;
    float ang_y;
    string timeField;
    int downsampleRate;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    int imuHz;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool loopClosureEnableFlag;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer(std::string node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node(node_name, options)
    {
        this->declare_parameter<std::string>("PROJECT_NAME", "sam");
        this->get_parameter("PROJECT_NAME", PROJECT_NAME);

        this->declare_parameter<std::string>("robot_id", "roboat");
        this->get_parameter("robot_id", robot_id);

        this->declare_parameter<std::string>(PROJECT_NAME + ".pointCloudTopic", "points_raw");
        this->declare_parameter<std::string>(PROJECT_NAME + ".imuTopic", "imu_correct");
        this->declare_parameter<std::string>(PROJECT_NAME + ".odomTopic", "odometry/imu");
        this->declare_parameter<std::string>(PROJECT_NAME + ".gpsTopic", "odometry/gps");
        
        this->get_parameter(PROJECT_NAME + ".pointCloudTopic", pointCloudTopic);
        this->get_parameter(PROJECT_NAME + ".imuTopic", imuTopic);
        this->get_parameter(PROJECT_NAME + ".odomTopic", odomTopic);
        this->get_parameter(PROJECT_NAME + ".gpsTopic", gpsTopic);

        this->declare_parameter<bool>(PROJECT_NAME + ".useImuHeadingInitialization", false);
        this->declare_parameter<bool>(PROJECT_NAME + ".useGpsElevation", false);
        this->declare_parameter<float>(PROJECT_NAME + ".gpsCovThreshold", 2.0);
        this->declare_parameter<float>(PROJECT_NAME + ".poseCovThreshold", 25.0);
        
        this->get_parameter(PROJECT_NAME + ".useImuHeadingInitialization", useImuHeadingInitialization);
        this->get_parameter(PROJECT_NAME + ".useGpsElevation", useGpsElevation);
        this->get_parameter(PROJECT_NAME + ".gpsCovThreshold", gpsCovThreshold);
        this->get_parameter(PROJECT_NAME + ".poseCovThreshold", poseCovThreshold);

        this->declare_parameter<bool>(PROJECT_NAME + ".savePCD", false);
        this->declare_parameter<std::string>(PROJECT_NAME + ".savePCDDirectory", "/tmp/loam/");
        
        this->get_parameter(PROJECT_NAME + ".savePCD", savePCD);
        this->get_parameter(PROJECT_NAME + ".savePCDDirectory", savePCDDirectory);

        this->declare_parameter<int>(PROJECT_NAME + ".N_SCAN", 16);
        this->declare_parameter<int>(PROJECT_NAME + ".Horizon_SCAN", 1800);
        this->declare_parameter<std::string>(PROJECT_NAME + ".timeField", "time");
        this->declare_parameter<int>(PROJECT_NAME + ".downsampleRate", 1);
        this->declare_parameter<float>(PROJECT_NAME + ".ang_y", 1);
        
        this->get_parameter(PROJECT_NAME + ".N_SCAN", N_SCAN);
        this->get_parameter(PROJECT_NAME + ".Horizon_SCAN", Horizon_SCAN);
        this->get_parameter(PROJECT_NAME + ".timeField", timeField);
        this->get_parameter(PROJECT_NAME + ".downsampleRate", downsampleRate);
        this->get_parameter(PROJECT_NAME + ".ang_y", ang_y);

        this->declare_parameter<float>(PROJECT_NAME + ".imuAccNoise", 0.01);
        this->declare_parameter<float>(PROJECT_NAME + ".imuGyrNoise", 0.001);
        this->declare_parameter<float>(PROJECT_NAME + ".imuAccBiasN", 0.0002);
        this->declare_parameter<float>(PROJECT_NAME + ".imuGyrBiasN", 0.00003);
        this->declare_parameter<float>(PROJECT_NAME + ".imuGravity", 9.80511);
        this->declare_parameter<int>(PROJECT_NAME + ".imuHz", 500);
        this->declare_parameter<vector<double>>(PROJECT_NAME + ".extrinsicRot", vector<double>());
        this->declare_parameter<vector<double>>(PROJECT_NAME + ".extrinsicRPY", vector<double>());
        this->declare_parameter<vector<double>>(PROJECT_NAME + ".extrinsicTrans", vector<double>());
        
        this->get_parameter(PROJECT_NAME + ".imuAccNoise", imuAccNoise);
        this->get_parameter(PROJECT_NAME + ".imuGyrNoise", imuGyrNoise);
        this->get_parameter(PROJECT_NAME + ".imuAccBiasN", imuAccBiasN);
        this->get_parameter(PROJECT_NAME + ".imuGyrBiasN", imuGyrBiasN);
        this->get_parameter(PROJECT_NAME + ".imuGravity", imuGravity);
        this->get_parameter(PROJECT_NAME + ".imuHz", imuHz);
        this->get_parameter(PROJECT_NAME + ".extrinsicRot", extRotV);
        this->get_parameter(PROJECT_NAME + ".extrinsicRPY", extRPYV);
        this->get_parameter(PROJECT_NAME + ".extrinsicTrans", extTransV);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        this->declare_parameter<float>(PROJECT_NAME + ".edgeThreshold", 0.1);
        this->declare_parameter<float>(PROJECT_NAME + ".surfThreshold", 0.1);
        this->declare_parameter<int>(PROJECT_NAME + ".edgeFeatureMinValidNum", 10);
        this->declare_parameter<int>(PROJECT_NAME + ".surfFeatureMinValidNum", 100);
        
        this->get_parameter(PROJECT_NAME + ".edgeThreshold", edgeThreshold);
        this->get_parameter(PROJECT_NAME + ".surfThreshold", surfThreshold);
        this->get_parameter(PROJECT_NAME + ".edgeFeatureMinValidNum", edgeFeatureMinValidNum);
        this->get_parameter(PROJECT_NAME + ".surfFeatureMinValidNum", surfFeatureMinValidNum);

        this->declare_parameter<float>(PROJECT_NAME + ".odometrySurfLeafSize", 0.2);
        this->declare_parameter<float>(PROJECT_NAME + ".mappingCornerLeafSize", 0.2);
        this->declare_parameter<float>(PROJECT_NAME + ".mappingSurfLeafSize", 0.2);
        
        this->get_parameter(PROJECT_NAME + ".odometrySurfLeafSize", odometrySurfLeafSize);
        this->get_parameter(PROJECT_NAME + ".mappingCornerLeafSize", mappingCornerLeafSize);
        this->get_parameter(PROJECT_NAME + ".mappingSurfLeafSize", mappingSurfLeafSize);

        this->declare_parameter<float>(PROJECT_NAME + ".z_tollerance", FLT_MAX);
        this->declare_parameter<float>(PROJECT_NAME + ".rotation_tollerance", FLT_MAX);
        
        this->get_parameter(PROJECT_NAME + ".z_tollerance", z_tollerance);
        this->get_parameter(PROJECT_NAME + ".rotation_tollerance", rotation_tollerance);

        this->declare_parameter<int>(PROJECT_NAME + ".numberOfCores", 2);
        this->declare_parameter<double>(PROJECT_NAME + ".mappingProcessInterval", 0.15);
        
        this->get_parameter(PROJECT_NAME + ".numberOfCores", numberOfCores);
        this->get_parameter(PROJECT_NAME + ".mappingProcessInterval", mappingProcessInterval);

        this->declare_parameter<float>(PROJECT_NAME + ".surroundingkeyframeAddingDistThreshold", 1.0);
        this->declare_parameter<float>(PROJECT_NAME + ".surroundingkeyframeAddingAngleThreshold", 0.2);
        this->declare_parameter<float>(PROJECT_NAME + ".surroundingKeyframeDensity", 1.0);
        this->declare_parameter<float>(PROJECT_NAME + ".surroundingKeyframeSearchRadius", 50.0);
        
        this->get_parameter(PROJECT_NAME + ".surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        this->get_parameter(PROJECT_NAME + ".surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        this->get_parameter(PROJECT_NAME + ".surroundingKeyframeDensity", surroundingKeyframeDensity);
        this->get_parameter(PROJECT_NAME + ".surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

        this->declare_parameter<bool>(PROJECT_NAME + ".loopClosureEnableFlag", false);
        this->declare_parameter<int>(PROJECT_NAME + ".surroundingKeyframeSize", 50);
        this->declare_parameter<float>(PROJECT_NAME + ".historyKeyframeSearchRadius", 10.0);
        this->declare_parameter<float>(PROJECT_NAME + ".historyKeyframeSearchTimeDiff", 30.0);
        this->declare_parameter<int>(PROJECT_NAME + ".historyKeyframeSearchNum", 25);
        this->declare_parameter<float>(PROJECT_NAME + ".historyKeyframeFitnessScore", 0.3);
        
        this->get_parameter(PROJECT_NAME + ".loopClosureEnableFlag", loopClosureEnableFlag);
        this->get_parameter(PROJECT_NAME + ".surroundingKeyframeSize", surroundingKeyframeSize);
        this->get_parameter(PROJECT_NAME + ".historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        this->get_parameter(PROJECT_NAME + ".historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        this->get_parameter(PROJECT_NAME + ".historyKeyframeSearchNum", historyKeyframeSearchNum);
        this->get_parameter(PROJECT_NAME + ".historyKeyframeFitnessScore", historyKeyframeFitnessScore);

        this->declare_parameter<float>(PROJECT_NAME + ".globalMapVisualizationSearchRadius", 1e3);
        this->declare_parameter<float>(PROJECT_NAME + ".globalMapVisualizationPoseDensity", 10.0);
        this->declare_parameter<float>(PROJECT_NAME + ".globalMapVisualizationLeafSize", 1.0);
        
        this->get_parameter(PROJECT_NAME + ".globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        this->get_parameter(PROJECT_NAME + ".globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        this->get_parameter(PROJECT_NAME + ".globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

        usleep(100);
    }
    //得到投影到lidar的imu信息
    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu &imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr; // imu作为中间过度 输入是imu相对于世界的 乘上lidar相对于imu的（即imu2lidar）
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        /**
         * @brief modified
         * 
         */
        // rotate roll pitch yaw
        filter.MahonyAHRSupdateIMU(imu_out.angular_velocity.x, imu_out.angular_velocity.y, imu_out.angular_velocity.z, imu_out.linear_acceleration.x, imu_out.linear_acceleration.y, imu_out.linear_acceleration.z);
        Eigen::Quaterniond q_mahony(filter.getQuaternionW(), filter.getQuaternionX(), filter.getQuaternionY(), filter.getQuaternionZ());
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final;
        // q_final = q_mahony;
        if (q_from.w() == 0.0)
        {
            // cout<<"using mahony!"<<q_from.w()<<endl;
            q_final = q_mahony;
        }
        else
        {
            q_final = q_from * extQRPY;
        }
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1)
        // if(q.norm()<0.1)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }
};

template <typename T>
sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, T thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template <typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg->header.stamp).seconds();
}

template<typename T>
double stamp2Sec(T stamp)
{
    return rclcpp::Time(stamp).seconds();
}

template <typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

#endif