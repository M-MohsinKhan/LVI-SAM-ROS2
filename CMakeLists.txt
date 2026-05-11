cmake_minimum_required(VERSION 3.16)
project(lvi_sam)

######################
### Cmake flags
######################
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################
### Packages
######################
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Ceres REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(yaml-cpp REQUIRED)

######################
### Message generation
######################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

######################
### Include directories
######################
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

######################
### Dependencies list for ament
######################
set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  message_filters
)

######################
### visual odometry
######################
file(GLOB visual_feature_files
    "src/visual_odometry/visual_feature/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)
file(GLOB visual_odometry_files
    "src/visual_odometry/visual_estimator/*.cpp"
    "src/visual_odometry/visual_estimator/factor/*.cpp"
    "src/visual_odometry/visual_estimator/initial/*.cpp"
    "src/visual_odometry/visual_estimator/utility/*.cpp"
)
file(GLOB visual_loop_files
    "src/visual_odometry/visual_loop/*.cpp"
    "src/visual_odometry/visual_loop/utility/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DBoW/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DUtils/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DVision/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)

# AHRS Library (Mahony filter - modified for 6-axis IMU)
add_library(ahrs STATIC src/lidar_odometry/mahonyMine.cpp)
ament_target_dependencies(ahrs ${dependencies})

# Visual Feature Tracker
add_executable(${PROJECT_NAME}_visual_feature ${visual_feature_files})
ament_target_dependencies(${PROJECT_NAME}_visual_feature ${dependencies})
target_link_libraries(${PROJECT_NAME}_visual_feature ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_visual_feature "${cpp_typesupport_target}")

# Visual Odometry
add_executable(${PROJECT_NAME}_visual_odometry ${visual_odometry_files})
ament_target_dependencies(${PROJECT_NAME}_visual_odometry ${dependencies})
target_link_libraries(${PROJECT_NAME}_visual_odometry ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_visual_odometry "${cpp_typesupport_target}")

# Visual Loop
add_executable(${PROJECT_NAME}_visual_loop ${visual_loop_files})
ament_target_dependencies(${PROJECT_NAME}_visual_loop ${dependencies})
target_link_libraries(${PROJECT_NAME}_visual_loop ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_visual_loop "${cpp_typesupport_target}")

######################
### lidar odometry
######################

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${dependencies})
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam ahrs)
target_link_libraries(${PROJECT_NAME}_imuPreintegration "${cpp_typesupport_target}")

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/lidar_odometry/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection ${dependencies})
target_link_libraries(${PROJECT_NAME}_imageProjection ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ahrs)
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}")

# Feature Extraction
add_executable(${PROJECT_NAME}_featureExtraction src/lidar_odometry/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction ${dependencies})
target_link_libraries(${PROJECT_NAME}_featureExtraction ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ahrs)
target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}")

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/lidar_odometry/mapOptmization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptmization ${dependencies})
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam ahrs yaml-cpp)
target_link_libraries(${PROJECT_NAME}_mapOptmization "${cpp_typesupport_target}")

######################
### Install
######################
install(TARGETS
  ahrs
  ${PROJECT_NAME}_visual_feature
  ${PROJECT_NAME}_visual_odometry
  ${PROJECT_NAME}_visual_loop
  ${PROJECT_NAME}_imuPreintegration
  ${PROJECT_NAME}_imageProjection
  ${PROJECT_NAME}_featureExtraction
  ${PROJECT_NAME}_mapOptmization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
