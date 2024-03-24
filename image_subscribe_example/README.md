English| [简体中文](./README_cn.md)

Getting Started with image_subscribe_example
=======

# Intro

The image_subscribe_example package is used to display image messages published by a ROS2 Node. It supports ROS standard format and also supports subscribing via shared memory.

# Build
---
## Dependency

ROS packages:
- sensor_msgs
- hbm_img_msgs

The hbm_img_msgs package defines a custom image message format in the hobot_msgs package, used for image transmission in shared memory scenarios.

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Compilation

Supports compilation on X3 Ubuntu system and cross-compilation using Docker on PC. It also supports controlling package dependencies and functionalities through compilation options.

### Compilation Options

BUILD_HBMEM

- Shared memory transmission enable/disable switch, default is disabled (OFF). Enable it during compilation using the command ` -DBUILD_HBMEM=ON`.
- When enabled, compilation and execution will depend on the hbm_img_msgs package and require the use of tros during compilation.
- When disabled, compilation and execution do not depend on the hbm_img_msgs package, supporting compilation using native ROS and tros.
- For shared memory communication, subscribing to compressed topics is not supported.
- The CMakeLists.txt specifies the installation path of the Mipi_cam package, defaulting to `../install/image_subscribe_example`.

### Compilation on X3 Ubuntu System

1. Compilation Environment Confirmation

- X3 Ubuntu system is installed on the board.
- The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`, where PATH is the installation path of TogetherROS.
- ROS2 compilation tool colcon is installed, installation command: `pip install -U colcon-common-extensions`
- Dependencies are installed, see the Dependency section for details.

2. Compilation:
   - Subscribe to images published via shared memory: `colcon build --packages-select image_subscribe_example --cmake-args -DBUILD_HBMEM=ON`
   This requires configuring the TROS environment first, for example: `source /opt/tros/setup.bash`
   - Support subscribing to ROS2 standard format images: `colcon build --packages-select image_subscribe_example` or `colcon build --packages-select image_subscribe_example --cmake-args -DBUILD_HBMEM=OFF`.### Cross-compilation with Docker

1. Environment Confirmation

- Compiled within docker, with tros installed in docker. For installation of docker, cross-compilation instructions, tros compilation and deployment instructions, please refer to the README.md in the robot development platform robot_dev_config repo.
- Package hbm_img_msgs has been compiled.

2. Compilation

- Compilation command:

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select image_subscribe_example \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake \
     -DBUILD_HBMEM=ON
  ```
- The SYS_ROOT is the path for cross-compilation system dependencies. Specific address of this path can be found in the cross-compilation instructions in step 1 "Environment Confirmation".

# Usage

## X3 Ubuntu System
Following successful compilation, copy the generated install path to the Horizon X3 development board (if compiled on X3, ignore the copying step), and execute the following command to run:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run image_subscribe_example subscribe_example
# Subscribe and save images by entering parameters: images are not saved by default unless the save_dir parameter is set
ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=/image_raw/compressed -p save_dir:=/userdata

Specify topic as hbmem_img, receiving data published by the publisher via share mem pub:
ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=hbmem_img
```

## X3 Linaro System

Copy the install directory cross-compiled in docker to the Linaro system, for example: /userdata
Firstly, specify the path for dependent libraries, for example:
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`

Modify the path for ROS_LOG_DIR, otherwise it will be created in the /home directory. Execute `mount -o remount,rw /` to create logs in /home.`export ROS_LOG_DIR=/userdata/`

Run subscribe_example
```
// Default parameter method
/userdata/install/lib/image_subscribe_example/subscribe_example
// Parameter passing method
/userdata/install/lib/image_subscribe_example/subscribe_example --ros-args -p sub_img_topic:=hbmem_img

```