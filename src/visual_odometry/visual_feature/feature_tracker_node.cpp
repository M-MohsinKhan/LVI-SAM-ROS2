//图像处理，特征点提取
// Lidar数据获取
#include "feature_tracker.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#define SHOW_UNDISTORTION 0

// mtx lock for two threads
std::mutex mtx_lidar;
// 将5s内的点云放在一起, 并进行体素滤波采样
// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue; // 保留5s内的激光点云 (world系)
deque<double> timeQueue;

// global depth register for obtaining depth of a feature
DepthRegister *depthRegister;

// global node pointer
rclcpp::Node::SharedPtr node_ptr;

// feature publisher for VINS estimator
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud2;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud3;

// TF2 listener
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// feature tracker variables
FeatureTracker trackerData[NUM_OF_CAM]; // 一般为单目, NUM_OF_CAM为1
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    double cur_img_time = rclcpp::Time(img_msg->header.stamp).seconds();

    if (first_image_flag) // 第一帧
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // detect unstable camera stream
    // 1.通过时间间隔判断相机数据流是否稳定，有问题则restart
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        RCLCPP_WARN(node_ptr->get_logger(), "image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    // 2.发布频率控制, 20Hz
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }
    // 3.图像格式转换(ROS to OpenCV)
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    // 4.主要操作: 读取图像数据 (光流跟踪, 提取特征)
    for (int i = 0; i < NUM_OF_CAM; i++) // 分别对每个相机处理
    {
        RCLCPP_DEBUG(node_ptr->get_logger(), "processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)                                                             // 读取第一张图片的数据 (单目, 双目)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_img_time); //读取原图像 同时提取线段特征
        else                                                                                     // 读取第二张图片的数据 (双目)
        {
            // 第二张图片不需要提取特征, 只需要进行均衡化处理
            if (EQUALIZE) // clahe 用于直方图均衡
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }
    // 5.更新当前帧新提取的特征点id
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i); // 更新特征的id
        if (!completed)
            break;
    }
    // 6.将当前帧特征发布出去, 供estimator_node使用
    if (PUB_THIS_FRAME)
    {
        // 1.将需要发布的数据都封装在一起 (归一化坐标, 特征点id, 像素坐标, 特征点速度)
        pub_count++;
        auto feature_points = std::make_shared<sensor_msgs::msg::PointCloud>(); // 待发布的特征点 (归一化坐标xy1)
        // 下面的数据加入到feature_points的通道中, 封装在一起 (后面还有深度通道)
        sensor_msgs::msg::ChannelFloat32 id_of_point; // 特征点id
        sensor_msgs::msg::ChannelFloat32 u_of_point;  // 特征点像素坐标 (uv)
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point; // 特征点速度 (归一化平面坐标xy1)
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header.stamp = img_msg->header.stamp;
        feature_points->header.frame_id = "vins_body"; //即imu坐标系

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++) // 单目
        {
            auto &un_pts = trackerData[i].cur_un_pts;         // 去畸变后的归一化坐标 xy1
            auto &cur_pts = trackerData[i].cur_pts;           // 像素坐标系 uv
            auto &ids = trackerData[i].ids;                   // 特征点id
            auto &pts_velocity = trackerData[i].pts_velocity; // 特征点速度
            // 遍历每一个特征点
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p; // 归一化坐标 xy1
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i); // 特征点id
                    u_of_point.values.push_back(cur_pts[j].x);           // 像素坐标
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x); // 特征点速度
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        // 2.从激光点云中获取特征点的深度
        // get feature depth from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud; // 5s内的激光点云
        mtx_lidar.unlock();
        sensor_msgs::msg::ChannelFloat32 depth_of_points = depthRegister->get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points->points);
        feature_points->channels.push_back(depth_of_points);
        // 3.发布当前帧的特征点云, 给其他node使用
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature->publish(*feature_points);
        // 4.在图像上标记提取到的特征点, 发布出去用于显示
        // publish features in image
        if (pub_match->get_subscription_count() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            // cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    }
                    else
                    {
                        // depth
                        if (j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }

            pub_match->publish(*ptr->toImageMsg());
        }
    }
}

void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg)
{
   
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP + 1) != 0)
        return; // 并不是所有激光帧都要
    // 0. listen to transform 获取lidar在世界坐标系的位姿(lidar to world)
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_->lookupTransform("vins_world", "vins_body_ros", 
                                                laser_msg->header.stamp,
                                                rclcpp::Duration::from_seconds(0.01)); // get wolrd^T_lidar
    }
    catch (tf2::TransformException &ex)
    {
        // RCLCPP_ERROR(this->get_logger(), "lidar no tf");
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform.transform.translation.x;
    yCur = transform.transform.translation.y;
    zCur = transform.transform.translation.z;
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur); // get wolrd^T_lidar

    // 1. convert laser cloud message to pcl 激光点云数据格式转换
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in); //原始的lidar信息

    // 2. downsample new cloud (save memory) 激光点云采样
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;
    /**
     * @brief modified
     * Take my device as example, lidar x--->left y--->right z---->up
     */
    // make the orientation in a normal way
    Eigen::Affine3f cam_R_lidar_original = pcl::getTransformation(0, 0, 0, C_RX_L, C_RY_L, C_RZ_L);
    // to rotate lidar to a normal axis direction (x-->front y--->left  z--->up)
    // and assume camera is in a normal direction (z--->front x--->right y--->down)
    Eigen::Matrix4f lidar_normal_R_cam;
    lidar_normal_R_cam << 0, 0, 1, 0,
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 0, 1;
    // point(lidar_original) ->cam  ===> point (lidar_normal)->cam change the representation of the points in lidar
    Eigen::Matrix4f lidar_normal_R_original;
    lidar_normal_R_original = lidar_normal_R_cam * cam_R_lidar_original.matrix();
    // 3. 剔除不在相机视野范围内的激光点
    // 3. filter lidar points (only keep points in camera view)

    // don't change the original lidar points
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        /**
         * @brief modified
         * 
         */
        PointType p_cam;
        p_cam.x = lidar_normal_R_original(0, 0) * p.x + lidar_normal_R_original(0, 1) * p.y + lidar_normal_R_original(0, 2) * p.z;
        p_cam.y = lidar_normal_R_original(1, 0) * p.x + lidar_normal_R_original(1, 1) * p.y + lidar_normal_R_original(1, 2) * p.z;
        p_cam.z = lidar_normal_R_original(2, 0) * p.x + lidar_normal_R_original(2, 1) * p.y + lidar_normal_R_original(2, 2) * p.z;
        if (p_cam.x >= 0 && abs(p_cam.y / p_cam.x) <= 10 && abs(p_cam.z / p_cam.x) <= 10) // assume lidar axis(x--->front y--->left z--->up)
            laser_cloud_in_filter->push_back(p);
    }
    
    // ROS_INFO("laser_cloud_in_filter size: %d",laser_cloud_in_filter->size());
    *laser_cloud_in = *laser_cloud_in_filter;
    // 4.  offset T_lidar -> T_camera 
    /**
     * @brief modified
     *
     */
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    Eigen::Affine3f transOffset = pcl::getTransformation(C_TX_L, C_TY_L, C_TZ_L,0, 0, 0);
    transOffset = transOffset.inverse();
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;
    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow); // get point to world
    // 6. save new cloud
    double timeScanCur = rclcpp::Time(laser_msg->header.stamp).seconds();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        }
        else
        {
            break;
        }
    }

    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
}

int main(int argc, char **argv)
{
    // initialize ROS2 node
    rclcpp::init(argc, argv);
    
    node_ptr = std::make_shared<rclcpp::Node>("feature_tracker_node");
    
    RCLCPP_INFO(node_ptr->get_logger(), "\033[1;32m----> Visual Feature Tracker Started.\033[0m");
    
    readParameters(node_ptr);

    // read camera params
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);
        trackerData[i].setNode(node_ptr);
    }

    // load fisheye mask to remove features on the boundry
    if (FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if (!trackerData[i].fisheye_mask.data)
            {
                RCLCPP_ERROR(node_ptr->get_logger(), "load fisheye mask fail");
                return -1;
            }
            else
                RCLCPP_INFO(node_ptr->get_logger(), "load mask success");
        }
    }

    // initialize depthRegister (after readParameters())
    depthRegister = new DepthRegister(node_ptr);

    // TF2 initialization
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // subscriber to image and lidar
    auto sub_img = node_ptr->create_subscription<sensor_msgs::msg::Image>(
        IMAGE_TOPIC, 5, img_callback);
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;
    if (USE_LIDAR)
    {
        sub_lidar = node_ptr->create_subscription<sensor_msgs::msg::PointCloud2>(
            POINT_CLOUD_TOPIC, 5, lidar_callback);
    }

    // messages to vins estimator
    pub_feature = node_ptr->create_publisher<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/feature/feature", 5);
    pub_match = node_ptr->create_publisher<sensor_msgs::msg::Image>(PROJECT_NAME + "/vins/feature/feature_img", 5);
    pub_restart = node_ptr->create_publisher<std_msgs::msg::Bool>(PROJECT_NAME + "/vins/feature/restart", 5);
    pub_cloud = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/vins/feature/cloud_test", 5);
    pub_cloud2 = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/vins/feature/cloud_test2", 5);
    pub_cloud3 = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/vins/feature/cloud_test3", 5);
    
    // use multi-threaded executor for parallel processing (image and lidar)
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_ptr);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}