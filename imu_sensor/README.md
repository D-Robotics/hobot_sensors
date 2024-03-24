English| [简体中文](./README_cn.md)

Getting Started with IMU Sensor Node
---
#Intro
The imu_sensor package is used to publish the sensor_msgs::msg::imu ROS2 topic, which includes the angular velocity and linear acceleration of object motion, as well as finely tuned timestamps.
This document details how to compile and use the imu_sensor package.
---

# Build

## Dependency

dependency libraries:
ros2 package:
- sensor_msgs
- rclcpp

## Developing Environment

- Language: C++
- Platform: X3
- Operating System: Ubuntu 20.04
- Compiling Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Package Description
Once the imu_sensor package is compiled, the config and launch directories, as well as the .so library, are installed in the
install/lib/imu_sensor or install/share/imu_sensor directory.

## Compiling
The package supports compiling on X3 board and cross-compiling on PC.
### Compiling on X3 in Ubuntu
1. Verify the compilation environment
- X3 with Ubuntu system
- Source the TogetheROS bash file `source $TogetheROS_PATH/setup.bash`, where $TogetheROS_PATH is the installation directory of TogetheROS
- Make sure colcon is installed, if not: `pip install -U colcon-common-extensions`

2. Compile:
   `colcon build --packages-select imu_sensor`.

### Cross compiling on PC in docker

1. Verify the compilation environment
- Refer to this document for installing the compilation environment
 (https://c-gitlab.horizon.ai/HHP/robot_dev_config/-/blob/develop/README.md)


2. Compilation

- Compilation command: 

  ``````bash
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
## X3 Ubuntu

Launched by 'ros2 run':

```bash
export COLCON_CURRENT_PREFIX=$YOUR_TROS_PATH
source $COLCON_CURRENT_PREFIX/setup.bash
ros2 run imu_sensor imu_sensor --ros-args -p config_file_path:=./install/lib/imu_sensor/config/bmi088.yaml
```

Launched by 'ros2 launch':

```bash
export COLCON_CURRENT_PREFIX=$YOUR_TROS_PATH
source $COLCON_CURRENT_PREFIX/setup.bash
ros2 launch imu_sensor imu_sensor.launch.py
```

Where config_file_path is the configuration file, the meanings of the fields in the configuration file such as i2c_bus, data range, and bandwidth are as follows.
```YAML
name: "bmi088"
# i2c_bus bus number
i2c_bus: 1
# Accelerometer range, unit 'g'
acc_range: 12
# Gyroscope range, unit 'deg/s'
gyro_range: 1000
# Accelerometer low-pass filter bandwidth
acc_bandwidth: 40
# Gyroscope low-pass filter bandwidth
gyro_bandwidth: 40
# group_delay of imu,
# which means the latency of the motion of body to data ready, unit 'ms'
group_delay: 7
# imu_data_path from which we read imu data
imu_data_path: "/dev/input/event2"
# imu_virtual_path from which we init imu
```imu_virtual_path: "/sys/devices/virtual/input/input0/"

---

## X3 linaro
Copy the install directory cross compiled by docker to X3 directory, 
such as /userdata. Then run command:
````
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`
````

Change ROS_LOG_DIR and run mount -o remount,rw /
```
export ROS_LOG_DIR=/userdata/
mount -o remount,rw /
```

launch imu_sensor
```
#/userdata/install/lib/imu_sensor/imu_sensor --ros-args -p config_file_path:=/userdata/install/lib/config/bmi088.yaml
```

Launched by 'ros2 launch':
`ros2 launch install/share/mipi_cam/launch/mipi_cam.launch.py`