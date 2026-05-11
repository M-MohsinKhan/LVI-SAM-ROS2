#include "visualization.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace Eigen;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry, pub_latest_odometry, pub_latest_odometry_ros;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud, pub_margin_cloud;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_pose;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
nav_msgs::msg::Path path;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_keyframe_point;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_extrinsic;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

void registerPub(rclcpp::Node::SharedPtr node)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1000));
    pub_latest_odometry     = node->create_publisher<nav_msgs::msg::Odometry>               (PROJECT_NAME + "/vins/odometry/imu_propagate", qos);
    pub_latest_odometry_ros = node->create_publisher<nav_msgs::msg::Odometry>               (PROJECT_NAME + "/vins/odometry/imu_propagate_ros", qos);
    pub_path                = node->create_publisher<nav_msgs::msg::Path>                   (PROJECT_NAME + "/vins/odometry/path", qos);
    pub_odometry            = node->create_publisher<nav_msgs::msg::Odometry>               (PROJECT_NAME + "/vins/odometry/odometry", qos);
    pub_point_cloud         = node->create_publisher<sensor_msgs::msg::PointCloud>          (PROJECT_NAME + "/vins/odometry/point_cloud", qos);
    pub_margin_cloud        = node->create_publisher<sensor_msgs::msg::PointCloud>          (PROJECT_NAME + "/vins/odometry/history_cloud", qos);
    pub_key_poses           = node->create_publisher<visualization_msgs::msg::Marker>       (PROJECT_NAME + "/vins/odometry/key_poses", qos);
    pub_camera_pose         = node->create_publisher<nav_msgs::msg::Odometry>               (PROJECT_NAME + "/vins/odometry/camera_pose", qos);
    pub_camera_pose_visual  = node->create_publisher<visualization_msgs::msg::MarkerArray>  (PROJECT_NAME + "/vins/odometry/camera_pose_visual", qos);
    pub_keyframe_pose       = node->create_publisher<nav_msgs::msg::Odometry>               (PROJECT_NAME + "/vins/odometry/keyframe_pose", qos);
    pub_keyframe_point      = node->create_publisher<sensor_msgs::msg::PointCloud>          (PROJECT_NAME + "/vins/odometry/keyframe_point", qos);
    pub_extrinsic           = node->create_publisher<nav_msgs::msg::Odometry>               (PROJECT_NAME + "/vins/odometry/extrinsic", qos);

    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}


tf2::Transform transformConversion(const geometry_msgs::msg::TransformStamped& t)
{
    tf2::Transform result;
    
    // This is the "magic" function that converts 
    // the message data into the math object correctly.
    tf2::fromMsg(t.transform, result); 
    
    return result;
}

// Static node holder for TF operations
static std::weak_ptr<rclcpp::Node> tf_node_weak;

void initializeTFforVisualization(rclcpp::Node::SharedPtr node) {
    tf_node_weak = node;
}

//输出predict处理后的P，V，Q
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header, const int &failureId)
{
    //此处的P V Q是imu
    static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    static double last_align_time = -1;

    // Initialize TF2 components if not already done and node is available
    if (!br && !tf_node_weak.expired()) {
        auto node = tf_node_weak.lock();
        if (node) {
            br = std::make_shared<tf2_ros::TransformBroadcaster>(node);
            tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        }
    }

    // Quternion not normalized
    if (Q.x() * Q.x() + Q.y() * Q.y() + Q.z() * Q.z() + Q.w() * Q.w() < 0.99)
    {
        // cout << "Q较小" << endl;
        return;
    }

    // imu odometry in camera frame
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.child_frame_id = "vins_body";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    // cout<<"imu部分的里程计:\n"<<P.x()<<" "<<P.y()<<" "<<P.z()<<endl;
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry->publish(odometry);

    odometry.pose.covariance[0] = double(failureId); // notify lidar odometry failure
    /**
     * @brief modified
     * world_T_imu represents world^T_imu
     * imu_T_lidar represents imu^T_lidar
     */
    // imu odometry in ROS format (change rotation), used for lidar odometry initial guess
    tf2::Transform world_T_imu(tf2::Quaternion(Q.x(), Q.y(), Q.z(), Q.w()), tf2::Vector3(P.x(), P.y(), P.z()));
    Eigen::Vector3d extTrans(L_TX_I, L_TY_I, L_TZ_I);
    tf2::Quaternion lidar_q_imu;
    lidar_q_imu.setRPY(L_RX_I, L_RY_I, L_RZ_I);
    tf2::Transform lidar_T_imu(lidar_q_imu, tf2::Vector3(extTrans.x(), extTrans.y(), extTrans.z()));
    tf2::Transform imu_T_lidar = lidar_T_imu.inverse();
    tf2::Transform world_T_lidar = world_T_imu * imu_T_lidar;
    // vins_world and odom are coincide because of global rotate
    nav_msgs::msg::Odometry test_odom;
    test_odom.header = header;
    test_odom.header.frame_id = "vins_world";
    test_odom.child_frame_id = "vins_body";
    test_odom.pose.pose.position.x = world_T_lidar.getOrigin().x();
    test_odom.pose.pose.position.y = world_T_lidar.getOrigin().y();
    test_odom.pose.pose.position.z = world_T_lidar.getOrigin().z();
    test_odom.pose.pose.orientation = tf2::toMsg(world_T_lidar.getRotation());
    pub_latest_odometry_ros->publish(test_odom); // lio部分接收的信息


    // TF of camera in vins_world in ROS format (change rotation), used for depth registration
    if (br) {
        geometry_msgs::msg::TransformStamped world_T_lidar_tf;
        world_T_lidar_tf.header.stamp = header.stamp;
        world_T_lidar_tf.header.frame_id = "vins_world";
        world_T_lidar_tf.child_frame_id = "vins_body_ros";
        world_T_lidar_tf.transform.translation.x = world_T_lidar.getOrigin().x();
        world_T_lidar_tf.transform.translation.y = world_T_lidar.getOrigin().y();
        world_T_lidar_tf.transform.translation.z = world_T_lidar.getOrigin().z();
        world_T_lidar_tf.transform.rotation = tf2::toMsg(world_T_lidar.getRotation());
        br->sendTransform(world_T_lidar_tf);
    }

    if (ALIGN_CAMERA_LIDAR_COORDINATE)
    {
        //获取lidar odom与vins world之间的转换
        // determine original rotation between odom and vins_world(check or debug it in rviz)
        // static tf2::Transform t_odom_world = tf2::Transform(tf2::createQuaternionFromRPY(0, 0, M_PI), tf2::Vector3(0, 0, 0));
        static tf2::Transform t_odom_world;
        static bool t_odom_world_initialized = false;
        if (!t_odom_world_initialized) {
            tf2::Quaternion q;
            q.setRPY(0, 0, M_PI);
            t_odom_world = tf2::Transform(q, tf2::Vector3(0, 0, 0));
            t_odom_world_initialized = true;
        }
        if (rclcpp::Time(header.stamp).seconds() - last_align_time > 1.0)
        {

            try
            {
                // Note: TF2 listener implementation would need to be added as a class member
                // For now, commenting out the transform lookup
                // geometry_msgs::msg::TransformStamped trans_odom_baselink;
                // listener.lookupTransform("odom", "base_link", tf2::TimePointZero, trans_odom_baselink);
                // t_odom_world = transformConversion(trans_odom_baselink) * transformConversion(world_T_lidar_tf).inverse();
                last_align_time = rclcpp::Time(header.stamp).seconds();
            }
            catch (const std::exception& ex)
            {
            }
        }
        if (br) {
            geometry_msgs::msg::TransformStamped t_odom_world_tf;
            t_odom_world_tf.header.stamp = header.stamp;
            t_odom_world_tf.header.frame_id = "odom";
            t_odom_world_tf.child_frame_id = "vins_world";
            t_odom_world_tf.transform.translation.x = t_odom_world.getOrigin().x();
            t_odom_world_tf.transform.translation.y = t_odom_world.getOrigin().y();
            t_odom_world_tf.transform.translation.z = t_odom_world.getOrigin().z();
            t_odom_world_tf.transform.rotation = tf2::toMsg(t_odom_world.getRotation());
            br->sendTransform(t_odom_world_tf);
        }
    }
    else
    {
        if (br) {
            tf2::Quaternion q_static;
            q_static.setRPY(0, 0, M_PI);
            tf2::Transform t_static(q_static, tf2::Vector3(0, 0, 0));
            geometry_msgs::msg::TransformStamped t_static_tf;
            t_static_tf.header.stamp = header.stamp;
            t_static_tf.header.frame_id = "odom";
            t_static_tf.child_frame_id = "vins_world";
            t_static_tf.transform.translation.x = t_static.getOrigin().x();
            t_static_tf.transform.translation.y = t_static.getOrigin().y();
            t_static_tf.transform.translation.z = t_static.getOrigin().z();
            t_static_tf.transform.rotation = tf2::toMsg(t_static.getRotation());
            br->sendTransform(t_static_tf);
        }
    }
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("visualization"), "position: " << estimator.Ps[WINDOW_SIZE].transpose());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("visualization"), "orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        // RCLCPP_DEBUG(rclcpp::get_logger("visualization"), "calibration result for camera %d", i);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("visualization"), "extirnsic tic: " << estimator.tic[i].transpose());
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("visualization"), "extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    RCLCPP_DEBUG(rclcpp::get_logger("visualization"), "vo solver costs: %f ms", t);
    RCLCPP_DEBUG(rclcpp::get_logger("visualization"), "average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    RCLCPP_DEBUG(rclcpp::get_logger("visualization"), "sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        RCLCPP_INFO(rclcpp::get_logger("visualization"), "td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.child_frame_id = "vins_world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        // cout<<"vins odometry:\n"
        // << odometry.pose.pose.position.x<<" "
        // << odometry.pose.pose.position.y<<" "
        // << odometry.pose.pose.position.z<<endl;

        pub_odometry->publish(odometry);

        static double path_save_time = -1;
        if (rclcpp::Time(header.stamp).seconds() - path_save_time > 0.5)
        {
            path_save_time = rclcpp::Time(header.stamp).seconds();
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.header.frame_id = "vins_world";
            pose_stamped.pose = odometry.pose.pose;
            path.header = header;
            path.header.frame_id = "vins_world";
            path.poses.push_back(pose_stamped);
            pub_path->publish(path);
        }
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_key_poses->get_subscription_count() == 0)
        return;
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::msg::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "vins_world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::msg::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = rclcpp::Duration::from_seconds(0);
    key_poses.id = 0;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses->publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header)
{

    if (pub_camera_pose_visual->get_subscription_count() == 0)
        return;

    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose->publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_point_cloud->get_subscription_count() != 0)
    {
        sensor_msgs::msg::PointCloud point_cloud;
        point_cloud.header = header;
        point_cloud.header.frame_id = "vins_world";

        sensor_msgs::msg::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;

            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::msg::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            point_cloud.points.push_back(p);

            if (it_per_id.lidar_depth_flag == false)
                intensity_channel.values.push_back(0);
            else
                intensity_channel.values.push_back(1);
        }

        point_cloud.channels.push_back(intensity_channel); //红色的点表示密度等于0
        pub_point_cloud->publish(point_cloud);
    }

    // pub margined potin
    if (pub_margin_cloud->get_subscription_count() != 0)
    {
        sensor_msgs::msg::PointCloud margin_cloud;
        margin_cloud.header = header;
        margin_cloud.header.frame_id = "vins_world";

        sensor_msgs::msg::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 && it_per_id.solve_flag == 1)
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

                geometry_msgs::msg::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                margin_cloud.points.push_back(p);

                if (it_per_id.lidar_depth_flag == false)
                    intensity_channel.values.push_back(0);
                else
                    intensity_channel.values.push_back(1);
            }
        }

        margin_cloud.channels.push_back(intensity_channel);
        pub_margin_cloud->publish(margin_cloud);
    }
}

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;

    // Use a static broadcaster to avoid re-creating it every time (standard ROS pattern)
    static std::shared_ptr<tf2_ros::TransformBroadcaster> br;

    // Initialize TF2 broadcaster using the weak_ptr from the node initialization
    if (!br && !tf_node_weak.expired()) {
        auto node = tf_node_weak.lock();
        if (node) {
            br = std::make_shared<tf2_ros::TransformBroadcaster>(node);
        }
    }

    if (br) {
        // --- 1. BODY FRAME (vins_world -> vins_body) ---
        tf2::Transform transform_body;
        tf2::Quaternion q_body;
        
        // Get current state from estimator
        Vector3d correct_t = estimator.Ps[WINDOW_SIZE];
        Quaterniond correct_q(estimator.Rs[WINDOW_SIZE]);

        // Set Math Objects (Identical to ROS 1 logic)
        transform_body.setOrigin(tf2::Vector3(correct_t(0), correct_t(1), correct_t(2)));
        q_body.setW(correct_q.w());
        q_body.setX(correct_q.x());
        q_body.setY(correct_q.y());
        q_body.setZ(correct_q.z());
        transform_body.setRotation(q_body);

        // Convert Math Object to Message and Publish
        geometry_msgs::msg::TransformStamped transform_body_msg;
        transform_body_msg.header = header;
        transform_body_msg.header.frame_id = "vins_world";
        transform_body_msg.child_frame_id = "vins_body";
        transform_body_msg.transform = tf2::toMsg(transform_body);
        br->sendTransform(transform_body_msg);

        // --- 2. CAMERA FRAME (vins_body -> vins_camera) ---
        tf2::Transform transform_camera;
        tf2::Quaternion q_camera;

        // Set translation from extrinsic calibration
        transform_camera.setOrigin(tf2::Vector3(estimator.tic[0].x(),
                                                estimator.tic[0].y(),
                                                estimator.tic[0].z()));
        
        // Set rotation from extrinsic calibration
        Quaterniond camera_q_eigen{estimator.ric[0]};
        q_camera.setW(camera_q_eigen.w());
        q_camera.setX(camera_q_eigen.x());
        q_camera.setY(camera_q_eigen.y());
        q_camera.setZ(camera_q_eigen.z());
        transform_camera.setRotation(q_camera);

        // Convert Math Object to Message and Publish
        geometry_msgs::msg::TransformStamped transform_camera_msg;
        transform_camera_msg.header = header;
        transform_camera_msg.header.frame_id = "vins_body";
        transform_camera_msg.child_frame_id = "vins_camera";
        transform_camera_msg.transform = tf2::toMsg(transform_camera);
        br->sendTransform(transform_camera_msg);
    }

    // --- 3. EXTRINSIC ODOMETRY PUBLISHING ---
    // This publishes the extrinsic pose as an odometry message for monitoring
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    
    if (pub_extrinsic) {
        pub_extrinsic->publish(odometry);
    }
}

void pubKeyframe(const Estimator &estimator)
{
    if ((!pub_keyframe_pose || pub_keyframe_pose->get_subscription_count() == 0) && (!pub_keyframe_point || pub_keyframe_point->get_subscription_count() == 0))
        return;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        if (pub_keyframe_pose)
            pub_keyframe_pose->publish(odometry);

        sensor_msgs::msg::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
                geometry_msgs::msg::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::msg::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }
        }
        if (pub_keyframe_point)
            pub_keyframe_point->publish(point_cloud);
    }
}