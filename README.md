# LVI-SAM

A ROS2 repo of LiDAR-Visual-Inertial system intended for GNSS denied applications. LVI-SAM original repository was based on ROS1 and older ubuntu versions also have issues with calibration parameters of the sensor that lack adaptability. All those problems are now fixed in this repository along with migration on the latest ROS2 Jazzy and Ubuntu 24.

<p align='center'>
    <img src="./doc/demo.gif" alt="drawing" width="800"/>
</p>

---

## Dependency

- [ROS 2](https://docs.ros.org/en/jazzy/Installation.html) (Tested with ROS 2 Jazzy on Ubuntu 24.04 ARM64)

- [Eigen 3.4.0](https://gitlab.com/libeigen/eigen/-/releases/3.4.0) (C++ template library for linear algebra)
  ```bash
  wget -O ~/Downloads/eigen-3.4.0.tar.gz [https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz](https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)
  cd ~/Downloads/ && tar -xzf eigen-3.4.0.tar.gz
  cd eigen-3.4.0
  mkdir build && cd build
  cmake ..
  sudo make install
  ```
- [OpenCV 4.8.0](https://github.com/opencv/opencv/releases/tag/4.8.0) (Computer Vision library)

  ```
  sudo apt-get install libopencv-dev
  
  ```
- [PCL 1.14.0](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.14.0) (Point Cloud Library)

  ```
  sudo apt-get install libpcl-dev  

  ```

- [gtsam 4.0.2](https://github.com/borglab/gtsam/releases/tag/4.0.2) (Georgia Tech Smoothing and Mapping library)

  ```
  wget -O ~/Downloads/gtsam.zip [https://github.com/borglab/gtsam/archive/4.0.2.zip](https://github.com/borglab/gtsam/archive/4.0.2.zip)
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.2/
  mkdir build && cd build
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
  sudo make install -j4  

  ```

- [Ceres Solver 2.2.0](https://github.com/ceres-solver/ceres-solver/releases/tag/2.2.0) (C++ library for modeling and solving large, complicated optimization problems)

  ```
  sudo apt-get install -y libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
  wget -O ~/Downloads/ceres.tar.gz [https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.2.0.tar.gz](https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.2.0.tar.gz)
  cd ~/Downloads/ && tar -xzf ceres.tar.gz
  cd ~/Downloads/ceres-solver-2.2.0/
  mkdir build && cd build
  cmake ..
  sudo make install -j4  

  ```

---

## Compile

You can use the following commands to download and compile the package.

```
cd ~/ros2ws/src
git clone <ProjectURL>
cd ..
colcon build --packages-select lvi_sam --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---
If you are experiencing a freeze on your PC, you can use -j2 or -j4 parallel workers.
## Datasets

<p align='center'>
    <img src="./doc/Rover.jpeg" alt="drawing" width="600"/>
</p>

The data-gathering sensor suite includes: Robosense Helio-16 lidar, Fixposition Vision RTK2 for IMU and Monocular Camera.

---

## Run the package

1. Configure parameters:

```
Configure sensor parameters in the .yaml files in the ```config``` folder.
```

2. Run the launch file:
```
ros2 launch lvi_sam run.launch.py
```

3. Play existing bag files:
```
ros2 bag play yourbag/ 
``` 

---

## TODO

  - [ ] Outdoor testing 



## Acknowledgement

  - The original version is from [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM).
