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

#ifndef HOBOT_MIPI_CAP_HPP_
#define HOBOT_MIPI_CAP_HPP_

#include <vector>
#include <string>

namespace mipi_cam {

typedef struct {
  std::string sensor_type;
  int pipeline_idx;
  int width;
  int height;
  int fps;
} MIPI_CAP_INFO_ST;


class HobotMipiCap {
 public:
  HobotMipiCap(/* args */) {}
  virtual ~HobotMipiCap() {}

  // 初始化设备环境，如X3的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  virtual int initEnv(std::string sensor) = 0;

  // 初始化相关sensor的VIO pipeline；
  // 输入参数：sensor_name--sensor的名称。
  // 返回值：0，初始化成功；-1，初始化失败。
  virtual int init(MIPI_CAP_INFO_ST &info) = 0;

  // 反初始化相关sensor的VIO pipeline ；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  virtual int deinit() = 0;

  // 启动相关sensor的VIO pipeline的码流；
  // 返回值：0，启动成功；-1，启动失败。
  virtual int start() = 0;

  // 停止相关sensor的VIO pipeline的码流；
  // 返回值：0，停止成功；-1，停止失败。
  virtual int stop() = 0;

  // 判断设备是否支持遍历设备连接的sensor
  // 返回值：true,支持；false，不支持
  virtual bool has_list_sensor() = 0;

  // 遍历设备连接的sensor
  virtual std::vector<std::string> list_sensor() = 0;

  // 如果有 vps ，就 输出vps 的分层数据
  virtual int GetFrame(int nChnID, int* nVOutW, int* nVOutH,
              void* buf, unsigned int bufsize, unsigned int*) = 0;

  // 检测对应的pipeline是否已经打开；
  // 输入参数：pipeline_idx pipeline的group ID。
  // 返回值：true，已经打开；false，没有打开。
  virtual bool check_pipeline_opened(int pipeline_idx) = 0;
};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_HPP_
