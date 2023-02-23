// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define __STDC_CONSTANT_MACROS
#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cam.hpp"
#include "hobot_mipi_factory.hpp"

#include "sensor_msgs/distortion_models.hpp"

#include <errno.h>
#include <malloc.h>
#include <unistd.h>

#include <assert.h>
#include <fcntl.h> /* low-level i/o */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace mipi_cam {

class MipiCamIml : public MipiCam {
 public:
  MipiCamIml();
  ~MipiCamIml();

  // 初始化摄像机
  // 输入参数：para是node传入的参数，包括sensor类型、名称，图像的宽、高等等。
  // 返回值：0，初始化成功，-1，初始化失败。
  int init(struct NodePara &para);

  // 反初始化摄像机；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  int deinit();

  // 启动摄像机的码流；
  // 返回值：0，启动成功；-1，启动失败。
  int start();

  // 停止摄像机的码流；
  // 返回值：0，停止成功；-1，停止失败。
  int stop();

  // grabs a new image from the camera
  bool get_image(
    builtin_interfaces::msg::Time & stamp,
    std::string & encoding,
    uint32_t & height, uint32_t & width,
    uint32_t & step, std::vector<uint8_t> & data);

  // grabs a new hbmem's image hbmem from the camera
  bool get_image_mem(
    builtin_interfaces::msg::Time & stamp,
    std::array<uint8_t, 12> & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step,
    std::array<uint8_t, 6220800> & data, uint32_t & data_size);

  // gen camera calibration
  bool get_cam_calibration(sensor_msgs::msg::CameraInfo& cam_info,
                           const std::string &file_path);

  bool is_capturing();

 private:
  bool lsInit_;
  bool is_capturing_;
  std::shared_ptr<HobotMipiCap> mipiCap_ptr_;
  struct NodePara nodePare_;
};

std::shared_ptr<MipiCam> MipiCam::create_mipicam() {
  return std::make_shared<MipiCamIml>();
}

MipiCamIml::MipiCamIml()
    : lsInit_(false),
      is_capturing_(false) {
}

MipiCamIml::~MipiCamIml() {
  stop();
  deinit();
}



int MipiCamIml::init(struct NodePara &para) {
  if (lsInit_) {
    return 0;
  }
  memcpy(&nodePare_, &para, sizeof(nodePare_));
  auto board_type = get_board_type();

  // mipiCap_ptr_ = create_mipiCap(nodePare_.camera_name_);
  mipiCap_ptr_ = create_mipiCap(board_type);
  if (!mipiCap_ptr_) {
    ROS_printf("[%s]->cap %s create capture failture.\r\n",
       __func__, board_type);
    return -1;
  }
  MIPI_CAP_INFO_ST cap_info;
  cap_info.sensor_type = nodePare_.video_device_name_;
  cap_info.width = nodePare_.image_width_;
  cap_info.height = nodePare_.image_height_;
  cap_info.fps = nodePare_.framerate_;

  mipiCap_ptr_->initEnv(nodePare_.video_device_name_);
  if (mipiCap_ptr_->has_list_sensor()) {
    bool detect_device = false;
    auto mipicap_v = mipiCap_ptr_->list_sensor();
    for (std::string video_device_name_temp : mipicap_v) {
      if (video_device_name_temp.empty()) {  // 未检测到有video_device连接
        ROS_printf(
          "[%s]->cam %s No camera detected!"
          " Please check if camera is connected.\r\n",
          __func__);
        return -2;
      } else if (!strcasecmp(video_device_name_temp.c_str(),
          nodePare_.video_device_name_.c_str())) {
        // 与用户传入的video_device不一致，比较不区分大小写
        // 当检测到的sensor与用户传入的不一致时，打开检测到的sensor，并输出log提示用户
        ROS_printf(
          "[%s]->cam No %s video_device was detected,"
          " but mipi_cam detected %s,"
          "mipi_cam will open %s device!"
          " You can change the video_device parameter to %s in the "
          "'/opt/tros/share/mipi_cam/launch/mipi_cam.launch.py'",
          nodePare_.video_device_name_.c_str(),
          video_device_name_temp.c_str(),
          video_device_name_temp.c_str(),
          video_device_name_temp.c_str());
        detect_device = true;
        break;
      }
    }
    if (detect_device == false) {
      ROS_printf(
        "[%s]->cam %s No camera detected!"
        " Please check if camera is connected.\r\n",
        __func__, nodePare_.video_device_name_);
      return -3;
    }
  }
  int pipeline_id = 0;
  for (; pipeline_id < 8; pipeline_id++) {
    if (!mipiCap_ptr_->check_pipeline_opened(pipeline_id)) {
      break;
    }
  }
  if (pipeline_id >= 8) {
    ROS_printf("[%s]->cam 8 channel pipeline ID was used .\r\n", __func__);
    return -4;
  }
  cap_info.pipeline_idx = pipeline_id;
  if (mipiCap_ptr_->init(cap_info) != 0) {
    ROS_printf("[%s]->cap capture init failture.\r\n", __func__);
    return -5;
  }
  ROS_printf(
    "[%s]->cap %s init success.\r\n", __func__, nodePare_.video_device_name_);
  lsInit_ = true;
  return 0;
}

int MipiCamIml::deinit() {
  if (is_capturing()) {
    stop();
  }
  auto ret = mipiCap_ptr_->deinit();
  lsInit_ = false;
  return ret;
}

int MipiCamIml::start() {
  if (!lsInit_ || is_capturing_) {
    return -1;
  }
  int ret = 0;
  if (mipiCap_ptr_->start()) {
    ROS_printf("[%s]->cap capture start failture.\r\n", __func__);
  }

  ROS_printf("[%s]->w:h=%d:%d.\n",
              __func__,
              nodePare_.image_width_,
              nodePare_.image_height_);
  is_capturing_ = true;
  return ret;
}

int MipiCamIml::stop() {
  int ret = 0;
  if (is_capturing()) {
    ret = mipiCap_ptr_->stop();
  }
  is_capturing_ = false;
  return ret;
}

bool MipiCamIml::is_capturing() { return is_capturing_; }

bool MipiCamIml::get_image(builtin_interfaces::msg::Time &stamp,
                        std::string &encoding,
                        uint32_t &height,
                        uint32_t &width,
                        uint32_t &step,
                        std::vector<uint8_t> &data) {
  if (!is_capturing_) {
    ROS_printf(
      "[%s][%-%d] Camera isn't captureing", __FILE__, __func__, __LINE__);
    return false;
  }
  if ((nodePare_.image_width_ == 0) || (nodePare_.image_height_ == 0)) {
    ROS_printf(
      "Invalid publish width:%d height: %d! Please check the image_width "
      "and image_height parameters!",
      nodePare_.image_width_,
      nodePare_.image_height_);
    return false;
  }
  struct timespec time_start = {0, 0};
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }

  int data_size = nodePare_.image_width_ * nodePare_.image_height_ * 1.5;

  data.resize(data_size);  // step * height);

  if (mipiCap_ptr_->GetFrame(
          2,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(&data[0]),
          data_size,
          reinterpret_cast<unsigned int *>(&data_size)))
    return false;
  encoding = "nv12";
  clock_gettime(CLOCK_REALTIME, &time_start);
  stamp.sec = time_start.tv_sec;
  stamp.nanosec = time_start.tv_nsec;
  step = width;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  ROS_printf("[%s]->enc=%s,step=%d, w:h=%d:%d,sz=%d,start %ld->laps=%ld ms.\n",
              __func__,
              encoding.c_str(),
              step,
              width,
              height,
              data_size,
              msStart,
              msEnd - msStart);
  return true;
}

bool MipiCamIml::get_image_mem(
    builtin_interfaces::msg::Time &stamp,
    std::array<uint8_t, 12> &encoding,
    uint32_t &height,
    uint32_t &width,
    uint32_t &step,
    std::array<uint8_t, 6220800> &data,
    uint32_t &data_size) {
  if (!is_capturing_) {
    ROS_printf(
      "[%s][%-%d] Camera isn't captureing", __FILE__, __func__, __LINE__);
    return false;
  }
  if ((nodePare_.image_width_ == 0) || (nodePare_.image_height_ == 0)) {
    ROS_printf(
      "Invalid publish width:%d height: %d! Please check the image_width "
      "and image_height parameters!",
      nodePare_.image_width_,
      nodePare_.image_height_);
    return false;
  }
  // get the image
  struct timespec time_start = {0, 0};
  uint64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  if (mipiCap_ptr_->GetFrame(
          2,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(data.data()),
          6220800,
          reinterpret_cast<unsigned int *>(&data_size)))
    return false;
  memcpy(encoding.data(), "nv12", strlen("nv12"));
  clock_gettime(CLOCK_REALTIME, &time_start);
  stamp.sec = time_start.tv_sec;
  stamp.nanosec = time_start.tv_nsec;
  step = width;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  ROS_printf("[%s]->hbmem enc=%s,step=%d,sz=%d,start %ld->laps=%ld ms.\n",
              __func__,
              encoding.data(),
              step,
              data_size,
              msStart,
              msEnd - msStart);
  return true;
}

bool MipiCamIml::get_cam_calibration(sensor_msgs::msg::CameraInfo &cam_info,
                                  const std::string &file_path) {
  try {
    std::string camera_name;
    std::ifstream fin(file_path.c_str());
    if (!fin) {
      ROS_printf("Camera calibration file: %s not exist! Please make sure the "
          "calibration file path is correct and the calibration file exists!",
          file_path.c_str());
      return false;
    }
    YAML::Node calibration_doc = YAML::Load(fin);
    if (calibration_doc["camera_name"]) {
      camera_name = calibration_doc["camera_name"].as<std::string>();
    } else {
      camera_name = "unknown";
    }
    cam_info.width = calibration_doc["image_width"].as<int>();
    cam_info.height = calibration_doc["image_height"].as<int>();

    const YAML::Node &camera_matrix = calibration_doc["camera_matrix"];
    const YAML::Node &camera_matrix_data = camera_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_info.k[i] = camera_matrix_data[i].as<double>();
    }
    const YAML::Node &rectification_matrix =
        calibration_doc["rectification_matrix"];
    const YAML::Node &rectification_matrix_data = rectification_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_info.r[i] = rectification_matrix_data[i].as<double>();
    }
    const YAML::Node &projection_matrix = calibration_doc["projection_matrix"];
    const YAML::Node &projection_matrix_data = projection_matrix["data"];
    for (int i = 0; i < 12; i++) {
      cam_info.p[i] = projection_matrix_data[i].as<double>();
    }

    if (calibration_doc["distortion_model"]) {
      cam_info.distortion_model =
          calibration_doc["distortion_model"].as<std::string>();
    } else {
      cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      ROS_printf("Camera calibration file did not specify distortion model, "
                  "assuming plumb bob");
    }
    const YAML::Node &distortion_coefficients =
        calibration_doc["distortion_coefficients"];
    int d_rows, d_cols;
    d_rows = distortion_coefficients["rows"].as<int>();
    d_cols = distortion_coefficients["cols"].as<int>();
    const YAML::Node &distortion_coefficients_data =
        distortion_coefficients["data"];
    cam_info.d.resize(d_rows * d_cols);
    for (int i = 0; i < d_rows * d_cols; ++i) {
      cam_info.d[i] = distortion_coefficients_data[i].as<double>();
    }
    ROS_printf("[get_cam_calibration]->parse calibration file successfully");
    return true;
  } catch (YAML::Exception &e) {
    ROS_printf("Unable to parse camera calibration file normally:%s",
                e.what());
    return false;
  }
}

}  // namespace mipi_cam