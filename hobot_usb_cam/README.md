English| [简体中文](./README_cn.md)

# Hobot_USB_CAM

# Function Introduction

Obtain image data from USB camera and publish it through the image/hbmem_image topic.

# Compilation

## Dependency Libraries

ROS package:

- rclcpp
- sensor_msgs
- hbm_img_msgs
- v4l-utils

hbm_img_msgs is a custom message format used to publish shared memory type image data, defined in hobot_msgs.

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Compilation

Supports compilation on X3/X86 Ubuntu system and cross-compilation using Docker on X86 Ubuntu.

### X3/X86 Ubuntu Compilation

1. Compilation Environment Confirmation
   - Ubuntu system is Ubuntu 20.04.
   - The current compilation terminal has set TogetherROS environment variables: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
   - ROS2 compilation tool `colcon` is installed, installation command: `pip install -U colcon-common-extensions`

2. Compilation

Compilation command: `colcon build --packages-select hobot_usb_cam`

### X86 Ubuntu Docker Cross-Compilation

1. Compilation Environment Confirmation
   - Compilation in Docker, and TogetherROS is already installed in Docker. For docker installation, cross-compilation instructions, TogetherROS compilation, and deployment instructions, please refer to the README.md in the robot development platform robot_dev_config repository.

2. Compilation

   - Compilation command:```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select hobot_usb_cam \
  --merge-install \
  --cmake-force-configure \
  --cmake-args \
      --no-warn-unused-cli \
      -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake \
      -DTHIRDPARTY=ON \
      -DBUILD_TESTING:BOOL=OFF \
      -DCMAKE_BUILD_RPATH="`pwd`/build/poco_vendor/poco_external_project_install/lib/;`pwd`/build/libyaml_vendor/libyaml_install/lib/"
```

## Instructions

# Dependencies

Websocket receives image messages and intelligent results messages, matches them based on timestamps, and then outputs them for rendering on the web side. It can also display images separately.

Image messages support `sensor_msgs::msg::Image` and `shared_mem` type `hbm_img_msgs::msg::HbmMsg1080P` messages, which must be jpeg format data output by Hobot codec.

Intelligent results messages support `ai_msgs::msg::PerceptionTargets` type messages. The `header.stamp` field must match the timestamp of the corresponding image message. Websocket uses this field for message matching, and the width and height of the intelligent results must match the resolution of the received image.

Packages that this depends on include:

- mipi_cam: Launches mipi cam and publishes nv12 type image messages.
- hobot_codec: Encodes nv12 images published by mipi_cam into jpeg images required by Websocket.
- mono2d_body_detection: Receives nv12 format data, performs algorithm inference, and publishes perception messages for human bodies, heads, faces, and hands.

## Parameters

| Parameter    | Explanation         | Type    | Supported Configurations   | Required | Default Value       |
| ------------ | ------------------- | ------- | ---------------------------| -------- | ------------------- |
| frame_id     | Message identifier  | string  | Set frame_id name as needed | No      | "default_usb_cam"  |
| framerate    | Frame rate          | int     | Choose based on sensor support | No    | 30                  |
| image_height | Image height        | int     | Choose based on sensor support | No    | 640                 |
| image_width  | Image width         | int     | Choose based on sensor support | No    | 480                 |
| io_method    | IO method           | string  | mmap/read/userptr           | No      | "mmap"              |
| pixel_format | Pixel format        | string  | Currently only supports mjpeg | No      | "mjpeg"             |
| video_device | Device driver name  | string  | Device name usually /dev/videox | Yes     | "/dev/video0"       |
| zero_copy    | Enable "zero_copy"  | bool    | True/False                  | No      | "True"              |
| camera_calibration_file_path  | Path to camera calibration file | string | Configure according to the actual path of the camera calibration file | No | Empty |
```## Running

After the compilation is successful, copy the generated install path to the Horizon X3 development board (ignore the copying steps if compiling on X3/X86 Ubuntu), and execute the following commands to run:

### **X3/X86 Ubuntu**

source setup.bash

~~~shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
~~~

Run hobot_usb_cam using launch
~~~shell
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py
~~~
Run hobot_usb_cam using command
~~~shell
ros2 run hobot_usb_cam hobot_usb_cam --ros-args --log-level info --ros-args -p video_device:="/dev/video8"
~~~

Note: The video_device parameter needs to be configured according to the actual situation

### **Linux**

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

./install/lib/hobot_usb_cam/hobot_usb_cam --ros-args --log-level info --ros-args -p video_device:="/dev/video8"
```

## Attention

Currently, using "zero-copy" only supports three resolution images: 1920*1080, 960*540, and 640*480. If you need to use other resolutions, you need to create corresponding ROS messages by yourself.

When the configured resolution is not supported by the hardware, the nearest resolution will be automatically selected for image acquisition.

hobot_usb_cam does not have a default calibration file, you can specify it using the parameter camera_calibration_file_path. Camera intrinsic parameter topic name: /camera_info

# Result Analysis

## Result Display

If the camera calibration file is not specified, a warning message indicating the failure to publish camera information will appear, but it does not affect the publishing of image messages.[INFO] [1661864867.688989561] [hobot_usb_cam]: Set resolution to 640x480

[INFO] [1661864867.718194946] [hobot_usb_cam]: Set framerate to be 30

[WARN] [1661864867.949372256] [hobot_usb_cam]: Unable to publish camera info.

[INFO] [1661864867.949684008] [hobot_usb_cam]: publish image 640x480 encoding:2 size:82225

[WARN] [1661864867.981565221] [hobot_usb_cam]: Unable to publish camera info.

[INFO] [1661864867.981989001] [hobot_usb_cam]: publish image 640x480 encoding:2 size:82194

[WARN] [1661864868.017477066] [hobot_usb_cam]: Unable to publish camera info.

[INFO] [1661864868.017740158] [hobot_usb_cam]: publish image 640x480 encoding:2 size:82427

[WARN] [1661864868.049163958] [hobot_usb_cam]: Unable to publish camera info.

[INFO] [1661864868.049382305] [hobot_usb_cam]: publish image 640x480 encoding:2 size:82302

[WARN] [1661864868.081569634] [hobot_usb_cam]: Unable to publish camera info.

[INFO] [1661864868.082001830] [hobot_usb_cam]: publish image 640x480 encoding:2 size:88354

[WARN] [1661864868.117489411] [hobot_usb_cam]: Unable to publish camera info.

```
If the camera calibration parameter path is specified, the camera runs successfully and obtains the camera calibration file normally, the following information will be output
```

[INFO] [1661865235.667735376] [hobot_usb_cam]: [get_cam_calibration]->parse calibration file successfully
[INFO] [1661865235.925195867] [hobot_usb_cam]: Set resolution to 640x480

[INFO] [1661865235.954375056] [hobot_usb_cam]: Set framerate to be 30

[INFO] [1661865236.185936360] [hobot_usb_cam]: publish camera info.

[INFO] [1661865236.186272480] [hobot_usb_cam]: publish image 640x480 encoding:2 size:83446

[INFO] [1661865236.217417891] [hobot_usb_cam]: publish camera info.

[INFO] [1661865236.217865635] [hobot_usb_cam]: publish image 640x480 encoding:2 size:83342

[INFO] [1661865236.252895697] [hobot_usb_cam]: publish camera info.

[INFO] [1661865236.253140610] [hobot_usb_cam]: publish image 640x480 encoding:2 size:83542

[INFO] [1661865236.285348631] [hobot_usb_cam]: publish camera info.

[INFO] [1661865236.285770625] [hobot_usb_cam]: publish image 640x480 encoding:2 size:83430# Frequently Asked Questions