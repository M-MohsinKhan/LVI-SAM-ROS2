#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::SharedPtr> feature_buf;

// global variable saving the lidar odometry
deque<nav_msgs::msg::Odometry> odomQueue;
odometryRegister *odomRegister;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;
std::mutex m_odom;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

Eigen::Matrix3d lidar_Rot_imu;
Eigen::Vector3d lidar_Trans_imu;

/**
 * @brief modified
 * 
 * @param n 
 * @param name 
 */
void getParam(rclcpp::Node::SharedPtr &n, const std::string &name)
{
    std::vector<double> param_vec;
    if (!n->get_parameter(name, param_vec))
        RCLCPP_ERROR(n->get_logger(), "Failed to get parameter from server.");
    
    for (size_t i = 0, j = 0, k = 0; i < param_vec.size(); ++i)
    {
        if (param_vec.size() == 9)
        {
            if (i % 3 == 0 && i != 0)
                j++;
            k = i % 3;
            lidar_Rot_imu(j, k) = param_vec[i];
        }
        else
        {
            lidar_Trans_imu(i) = param_vec[i];
        }
    }
}

void predict(const sensor_msgs::msg::Imu::SharedPtr &imu_msg)
{
    double t = rclcpp::Time(imu_msg->header.stamp).seconds();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::msg::Imu::SharedPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msg::Imu::SharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;

    while (rclcpp::ok())
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(rclcpp::Time(imu_buf.back()->header.stamp).seconds() > rclcpp::Time(feature_buf.front()->header.stamp).seconds() + estimator.td))
        {
            return measurements;
        }

        if (!(rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(feature_buf.front()->header.stamp).seconds() + estimator.td))
        {
            RCLCPP_WARN(rclcpp::get_logger("estimator_node"), "throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud::SharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msg::Imu::SharedPtr> IMUs;
        while (rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCLCPP_WARN(rclcpp::get_logger("estimator_node"), "no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    if (rclcpp::Time(imu_msg->header.stamp).seconds() <= last_imu_t)
    {
        RCLCPP_WARN(rclcpp::get_logger("estimator_node"), "imu message in disorder!");
        return;
    }
    last_imu_t = rclcpp::Time(imu_msg->header.stamp).seconds();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = rclcpp::Time(imu_msg->header.stamp).seconds();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::msg::Header header = imu_msg->header;
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, estimator.failureCount);
    }
}

void odom_callback(nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    m_odom.lock();
    odomQueue.push_back(*odom_msg);
    m_odom.unlock();
}

void feature_callback(sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        RCLCPP_WARN(rclcpp::get_logger("estimator_node"), "restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// thread: visual-inertial odometry
void process()
{
    while (rclcpp::ok())
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();

        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;

            // 1. IMU pre-integration
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = rclcpp::Time(imu_msg->header.stamp).seconds();
                double img_t = rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // 2. VINS Optimization
            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                double depth = img_msg->channels[5].values[i];

                assert(z == 1);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
            }

            /**
             * @brief modified
             * import extrinsic from imu to lidar
             */
            // Get initialization info from lidar odometry
            vector<float> initialization_info;
            m_odom.lock();
            initialization_info = odomRegister->getOdometry(odomQueue, rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td, lidar_Rot_imu, lidar_Trans_imu);
            m_odom.unlock();


            estimator.processImage(image, initialization_info, img_msg->header);
            // double whole_t = t_s.toc();
            // printStatistics(estimator, whole_t);

            // 3. Visualization
            std_msgs::msg::Header header = img_msg->header;
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
        }
        m_estimator.unlock();

        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

// Global node pointer for callbacks
rclcpp::Node::SharedPtr node_ptr;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node_ptr = rclcpp::Node::make_shared("vins");
    RCLCPP_INFO(node_ptr->get_logger(), "\033[1;32m----> Visual Odometry Estimator Started.\033[0m");

    readParameters(node_ptr);
    estimator.setParameter();

    registerPub(node_ptr);
    initializeTFforVisualization(node_ptr);

    odomRegister = new odometryRegister(node_ptr);

    rclcpp::QoS qos_imu(5000);
    qos_imu.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    auto sub_imu     = node_ptr->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, qos_imu, imu_callback);
    auto sub_odom    = node_ptr->create_subscription<nav_msgs::msg::Odometry>("odometry/imu", 5000, odom_callback);
    auto sub_image   = node_ptr->create_subscription<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/feature/feature", 1, feature_callback);
    auto sub_restart = node_ptr->create_subscription<std_msgs::msg::Bool>(PROJECT_NAME + "/vins/feature/restart", 1, restart_callback);

    if (!USE_LIDAR) {
        sub_odom.reset(); // Equivalent to shutdown in ROS2
    }

    std::thread measurement_process{process};

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_ptr);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}