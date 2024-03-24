English| [简体中文](./README_cn.md)

# Getting Started with rgbd_sensor Node
---
# Intro
---
By reading this document, users can easily capture the video stream data of the MIPI depth camera on the Horizon X3 development board, and publish depth map data/grayscale image data/RGB image data/camera parameters that meet the ROS standard on the ROS platform, as well as computed point cloud data for other ROS Nodes to subscribe to and view real-time effects on rviz. Supports share mem mode of publishing.

# Build
---
## Dependency

Dependencies:
ros packages:
- sensor_msgs
- hbm_img_msgs

The hbm_img_msgs pkg is a custom image message format in hobot_msgs, used for image transmission in shared memory scenarios.

## Development Environment
- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0
## Package Description
---
The source code includes the rgbd_sensor package. After compiling rgbd_sensor, the header files, dynamic libraries, and dependencies are installed in the path install/rgbd_sensor.

## Compilation
Supports two ways of compilation: compiling on X3 Ubuntu system and cross-compiling using docker on a PC, and supports controlling the dependencies and functionalities of the package through compilation options.

### Compilation on X3 Ubuntu System
1. Environment Confirmation

- The X3 Ubuntu system is installed on the board.
- The current compilation terminal has set up the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
- ROS2 compilation tool colcon is installed, installation command: `pip install -U colcon-common-extensions`
- Required dependencies are installed, see the Dependency section for details.

2. Compilation:
   `colcon build --packages-select rgbd_sensor`.
   
### Cross-compilation using Docker
1. Environment Confirmation

- Compile in docker, and tros is already installed in the docker. For docker installation, cross-compilation instructions, and tros compilation and deployment instructions, refer to README.md in the robot development platform robot_dev_config repo.
- The hbm_img_msgs package has been compiled (see Compilation method in the Dependency section).

2. Compilation- Compilation command:

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select rgbd_sensor \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
   
  ```


# Usage

## Current parameter list:

| Parameter       | Meaning               | Values                             | Default               |
| -------------- | ---------------------   | ----------------------------------  | --------------------- |
| sensor_type    | Device type           | String, currently only supports Tuya CP3AM | CP3AM               |
| io_method      | Method of output data transfer | ros/shared_mem          | ros               |
| color_width    |  Color image width     | 1920                             | 1920           |
| color_height   |  Color image height    | 1080                             | 1080           |
| color_fps      |  Color image frame rate | 10                              | 10            |
| enable_color   |  Whether to publish images | True/False                  | True          |
| depth_width    |  Depth image width    | 224                               | 224           |
| depth_height   |  Depth image height    | 129                               | 129           |
| depth_fps      |  Depth image frame rate | 10                               | 10            |
| enable_depth   |  Whether to publish depth images | True/False            | True          |
| enable_pointcloud |  Whether to publish point cloud | True/False            | True          |
| enable_aligned_pointcloud |  Whether to publish aligned point cloud | True/False   | True          |
| infra_width    |  Infrared image width   | 224                              | 224           |
| infra_height   |  Infrared image height   | 108                              | 108           |
| infra_fps      |  Infrared image frame rate | 10                             | 10            |
| enable_infra   |  Whether to publish infrared images | True/False            | True          |
| camera_calibration_file_path  | Path to store the camera calibration file | Configure based on the actual path of the camera calibration file | /opt/tros/lib/rgbd_sensor/config/CP3AM_calibration.yaml |

Currently, Tuya modules can only output 1080P calibration, so the image parameters currently have no practical effect and are all default values.
Published topics include:
```
#ros
#Depth image
``````/rgbd_CP3AM/depth/image_rect_raw
# Point cloud
/rgbd_CP3AM/depth/color/points
# Calibration point cloud
/rgbd_CP3AM/aligned_depth_to_color/color/points
# Grayscale image
/rgbd_CP3AM/infra/image_rect_raw
# Color image
/rgbd_CP3AM/color/image_rect_raw
# Camera parameters
/rgbd_CP3AM/color/camera_info
# Shared memory:
# Color image
hbmem_img
# Depth image
hbmem_depth
# Grayscale image
hbmem_infra
```

## Note:
1: In the current directory, copy -r install/${PKG_NAME}/lib/${PKG_NAME}/parameter/ ., where ${PKG_NAME} is the specific package name.

2: The calibration library install/lib/libgc2053_linear.so needs to be copied to: /lib/sensorlib/.

3: If the camera parameter file reading fails, a warning about being unable to publish camera parameters will appear, but it does not affect other functions of rgbd_sensor.

4: The default reading path for the camera calibration file is: /opt/tros/lib/rgbd_sensor/config/CP3AM_calibration.yaml. Please confirm the file path is correct, and the camera parameter publishing topic name is: /rgbd_CP3AM/color/camera_info.

## X3 Ubuntu System
Users can directly start by using the ros2 command:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh

ros2 run rgbd_sensor rgbd_sensor
```
Passing parameters:
`ros2 run rgbd_sensor rgbd_sensor --ros-args --log-level info --ros-args -p io_method:=ros`

Another way to run is by using a launch file:
`ros2 launch install/share/rgbd_sensor/launch/rgbd_sensor.launch.py`

## X3 Linaro System
Copy the install directory compiled in the docker environment to the Linaro system, for example: /userdata.
Specify the path to the dependency library first, for example:
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`Please modify the path of ROS_LOG_DIR, otherwise it will be created under the /home directory. You need to execute `mount -o remount,rw /` to be able to create logs under /home.
`export ROS_LOG_DIR=/userdata/`

Run rgbd_sensor
```
// Default parameter method
/userdata/install/lib/rgbd_sensor/rgbd_sensor
// Parameter passing method
#/userdata/install/lib/rgbd_sensor/rgbd_sensor --ros-args -p io_method:=ros

```
# Results Analysis
## X3 Result Display
If the camera runs normally and successfully publishes camera parameters, the following information will be output
```
[INFO] [1662470301.451981102] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.461459373] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.528991765] [rgbd_node]: publish camera info.

[INFO] [1662470301.533941164] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24193.
[INFO] [1662470301.543212650] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24193.
[INFO] [1662470301.608683161] [rgbd_node]: publish camera info.

[INFO] [1662470301.613038915] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24194.
[INFO] [1662470301.621678967] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24194.
[INFO] [1662470301.713615364] [rgbd_node]: publish camera info.

[INFO] [1662470301.717811416] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24195.
[INFO] [1662470301.726191436] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24195.
[INFO] [1662470301.818197996] [rgbd_node]: publish camera info.

[INFO] [1662470301.822495336] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24196.
[INFO] [1662470301.831028892] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24196.
[INFO] [1662470301.909490084] [rgbd_node]: publish camera info.

[INFO] [1662470301.913704051] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24197.
[INFO] [1662470301.922111820] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24197.
[INFO] [1662470302.014477201] [rgbd_node]: publish camera info.
```

## rviz2 Visualization
Install: apt install ros-foxy-rviz-common ros-foxy-rviz-default-plugins ros-foxy-rviz2