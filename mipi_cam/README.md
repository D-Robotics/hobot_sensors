English| [简体中文](./README_cn.md)

# Getting Started with Mipi_Cam Node
---
# Intro
---
By reading this document, users can easily capture video streaming data from mipi camera on Horizon X3 development board, and publish image data that meets ROS standards through the ROS platform for other ROS Nodes to subscribe. Currently supported mipi standard devices include F37, IMX415, GC4663, IMX219, IMX477, OV5647, etc.
Mipi_cam Node package is part of Horizon Robotics Robot Development Platform, based on Horizon VIO and ROS2 Node for secondary development, providing simple and easy-to-use camera data acquisition functions for application development, avoiding repetitive work of acquiring video. Support for publishing via shared mem.

# Build
---
## Dependency

Dependencies:
ros package:
- sensor_msgs
- hbm_img_msgs

The hbm_img_msgs package is a custom image message format defined in hobot_msgs, used for image transmission in shared mem scenarios.

## Development Environment
- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0
## Package Description
---
The source code includes the mipi_cam package. After compilation of mipi_cam is completed, header files, dynamic libraries, and dependencies are installed in the install/mipi_cam path.

## Compilation
Supports compilation on X3 Ubuntu system and cross-compilation using docker on PC, and supports controlling the dependencies and functionality of compiling pkg through compilation options.

### Compilation on X3 Ubuntu System
1. Confirmation of compilation environment

- X3 Ubuntu system is installed on the board.
- The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
- ROS2 compilation tool colcon is installed, installation command: `pip install -U colcon-common-extensions`
- Dependencies are installed, see Dependency section for details

2. Compilation:
  `colcon build --packages-select mipi_cam`.


### Cross-Compilation using Docker

1. Confirmation of compilation environment

- Compilation in docker, and tros is already installed in docker. For docker installation, cross-compilation instructions, tros compilation and deployment instructions, please refer to the README.md in the robot development platform robot_dev_config repository.
- The hbm_img_msgs package has been compiled (compilation method see Dependency section).

2. Compilation
- Compilation command:

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select mipi_cam \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
     
  ```
  



# Usage
## X3 Ubuntu System

To run, users can directly use the ros2 command:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# Default F37 sensor
ros2 run mipi_cam mipi_cam
```

When using the shared memory communication method, only publishing images of the 'hbmem_img' topic is supported.

The node will publish on the '/image_raw' topic with images in rgb8 format and publish the 'hbmem_img' topic using shared memory. Camera intrinsic parameters are published on the '/camera_info' topic.

One can use 'rqt_image_view' to view the published image topic, as well as use image consumer nodes. For example, one can directly obtain images for inference and other applications using the examples in this repository.

You can configure the sensor to use, the encoding method, and the resolution for publishing images.

Use the 'video_device' parameter to set the sensor to use. Currently, F37 (default) and IMX415 are supported (set through `--ros-args -p video_device:=IMX415`). The default resolution for F37 is 1920x1080, and for IMX415 it is 3840x2160.

Use the 'image_width' and 'image_height' parameters to set the resolution for publishing images:

`ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p image_width:=960 -p image_height:=540 -p video_device:=F37`

Use the 'out_format' parameter to set the encoding method for publishing images, default is 'bgr8'. Support is provided for the nv12 format (/image_raw topic). For example, to publish nv12 format images of 960x540 resolution using the F37 sensor:

`ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=540 -p video_device:=F37`  Use the `io_method` parameter to set the way the image is published. Currently, the topic published by `shared_mem` is fixed as: `hbmem_img`.

```bash
ros2 run mipi_cam mipi_cam --ros-args -p io_method:=shared_mem
```

Set the camera calibration file path using the `camera_calibration_file_path` parameter. Here is an example using the GC4663 camera and reading the `GC4663_calibration.yaml` file under the `config` directory (see Attention below for print information):

```bash
# Copy the camera calibration file provided in the config as an example, copy according to the actual installation path
cp -r install/lib/mipi_cam/config/ .
ros2 run mipi_cam mipi_cam --ros-args -p camera_calibration_file_path:=./config/GC4663_calibration.yaml -p video_device:=GC4663
```

---

## X3 Linaro System

Copy the `install` directory cross-compiled in Docker to the Linaro system, for example: `/userdata`. It is necessary to specify the path of the dependent library first, for example:
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`

Modify the path of `ROS_LOG_DIR`, otherwise it will be created under the `/home` directory. You need to execute `mount -o remount,rw /` to create logs under `/home`.
`export ROS_LOG_DIR=/userdata/`

Run the `mipi_cam` node:
```bash
// Default parameter mode
/userdata/install/lib/mipi_cam/mipi_cam
// Parameter mode
#/userdata/install/lib/mipi_cam/mipi_cam --ros-args -p image_width:=960 -p image_height:=540
```

Run mode 2, use launch file to start:
`ros2 launch install/share/mipi_cam/launch/mipi_cam.launch.py`

# Attention
Currently, the data output from the device is in the nv12 format, needs conversion to rgb8 format. Currently not using OpenCV, performance time for 1920*1080 image is around 100ms, compressed images need to be supported in a relay manner:
```bash
ros2 run image_transport republish [in_transport] in:=<in_base_topic> [out_transport] out:=<out_base_topic>
e.g.:
ros2 run image_transport republish raw compressed --ros-args --remap in:=/image_raw --remap compressed:=/image_raw/compressed
Then there will be a `compressed` topic, use the sub end to subscribe to the compressed image topic, for example:
ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=/image_raw/compressed
Log display:
```
```bash
root@xj3ubuntu:/userdata/cc_ws/tros_ws# ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=/image_raw/compressed
[WARN] [1648302887.615608845] [example]: This is image_subscriber example!
[WARN] [1648302887.699318639] [ImageSubscriber]: Update sub_img_topic with topic_name: /image_raw/compressed
[WARN] [1648302887.701353516] [ImageSubscriber]: Update save_dir: 
[WARN] [1648302887.701502469] [ImageSubscriber]: Create subscription with topic_name: /image_raw/compressed
```[WARN] [1648302887.705133283] [example]: ImageSubscriber init!
[WARN] [1648302887.706179033] [example]: ImageSubscriber add_node!
[INFO] [1648302889.318928227] [img_sub]: Recv compressed img
[WARN] [1648302889.319329711] [img_sub]: Sub compressed img fps = 1
[INFO] [1648302889.319478247] [img_sub]: Recv compressed img: rgb8; jpeg compressed bgr8, stamp: 1648302889.92334955, tmlaps(ms): 227, data size: 33813

```
To enable this feature, it is necessary to install the ROS package image_transport_plugins using the following command:
sudo apt-get install ros-foxy-image-transport-plugins

If the camera runs successfully and reads the camera calibration file normally, the following information will be output. mipi_cam provides calibration files for two camera models, GC4663 and F37. By default, it reads the F37_calibration.yaml file in the config folder. If using the GC4663 camera, please change the path to read the camera calibration file!
```

[INFO] [1661863164.454533227] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1544567->laps=102 ms.

[INFO] [1661863164.458727776] [mipi_node]: publish camera info.

[INFO] [1661863164.562431009] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1544674->laps=103 ms.

[INFO] [1661863164.566239194] [mipi_node]: publish camera info.

[INFO] [1661863164.671290847] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1544782->laps=104 ms.

[INFO] [1661863164.675211155] [mipi_node]: publish camera info.

[INFO] [1661863164.780465260] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1544891->laps=104 ms.

[INFO] [1661863164.784429400] [mipi_node]: publish camera info.

[INFO] [1661863164.887891555] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1545000->laps=103 ms.

[INFO] [1661863164.891738656] [mipi_node]: publish camera info.

[INFO] [1661863164.994701993] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1545107->laps=103 ms.

[INFO] [1661863164.998455013] [mipi_node]: publish camera info.

[INFO] [1661863165.100916367] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1545214->laps=102 ms.

[INFO] [1661863165.104211776] [mipi_node]: publish camera info.