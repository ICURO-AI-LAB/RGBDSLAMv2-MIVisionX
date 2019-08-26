# RGBDSLAMv2-MIVisionX

This is an implementation of [RGBDSLAM_V2](https://github.com/felixendres/rgbdslam_v2) that utilizes AMD MIVisionX for feature detection and ROCm OpenCL for offloading computations to Radeon GPUs.

## Prerequisites:

### Linux

* [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
* [Radeon Open Compute (ROCm)](https://rocm.github.io/ROCmInstall.html)
* [AMD MIVisionX](https://github.com/GPUOpen-ProfessionalCompute-Libraries/MIVisionX)
* [OpenCV 3.4](https://github.com/opencv/opencv/releases/tag/3.4.0)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)

Follow the steps in [INSTRUCTIONS_PREREQUISITES.md](INSTRUCTIONS_PREREQUISITES.md) to install the prerequisites of RGBDSLAM_v2 on ROS Kinetic.

* [Forked version of g2o](https://github.com/felixendres/g2o)
* [Eigen 3.2.10](https://bitbucket.org/eigen/eigen/src)
* [PCL 1.8](https://github.com/PointCloudLibrary/pcl)

## Build & Install RGBDSLAM_v2
After installing the prerequisites, use these instructions to install and run RGBDSLAM_v2.

* Make a catkin workspace and clone this repo inside the source folder of the workspace
* Use catkin_make to build

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/ICURO-AI-LAB/RGBDSLAMv2-MIVisionX.git
cd ..
catkin_make
```

## Running RGBDSLAM_v2
* ROSbag
```
source catkin_ws/devel/setup.bash
roslaunch rgbdslam test_settings.launch bagfile_name:=<path/to/rosbag>
```
* Live
```
source catkin_ws/devel/setup.bash
roslaunch rgbdslam rgbdslam.launch
```
## Sample ROS bags
Sample ROS bags for quickly testing the install can be found [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)

## Docker:
Scripts for building and running a docker image is provided in this directory. This can be used to easily install RGBDSLAM_v2 without dependency issues.

* Dockerfile.rgbdslam builds an image with all the prerequisites installed
* Build
```bash
cd catkin_ws/src/rgbdslam_mivisionx/docker
./build
```

* Run
```bash
cd catkin_ws/src/rgbdslam_mivisionx/docker
./run
```

* Build rgbdslamv2_mivisionx in docker container
```
cd ~;
git clone https://github.com/GPUOpen-ProfessionalCompute-Libraries/MIVisionX.git;
cd MIVisionX/apps/rgbdslam_v2;
catkin_make;
```
