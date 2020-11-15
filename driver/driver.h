/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/robosense/proto/config.pb.h"
#include "modules/drivers/robosense/proto/robosense.pb.h"
#include "modules/drivers/robosense/proto/pointcloud_withfullinfo.pb.h"

#include "cyber/cyber.h"
#include "cyber/base/thread_pool.h"
#include "modules/drivers/robosense/decoder/decoder_16.hpp"
#include "modules/drivers/robosense/decoder/decoder_factory.hpp"
#include "modules/drivers/robosense/driver/utility.h"
#include "modules/drivers/robosense/input/input.h"
#define PKT_DATA_LENGTH 1248
namespace apollo {
namespace drivers {
namespace robosense {
struct alignas(16) LidarPacketMsg {
  double timestamp = 0.0;
  std::string frame_id = "";
  std::array<uint8_t, PKT_DATA_LENGTH> packet{};  ///< lidar single packet
};

struct PointXYZITR_RS{
    float x;
    float y;
    float z;
    float intensity;
    uint64_t timestamp;
    uint16_t ring;

    void set_x(float value){ x = value;}
    void set_y(float value){ y = value;}
    void set_z(float value){ z = value;}
    void set_intensity(float value){ intensity = value;}
    void set_timestamp(uint64_t value){ timestamp = value;}
};


class RobosenseDriver {
 public:
  RobosenseDriver(const std::shared_ptr<cyber::Node> &node, const Config &conf)
      : node_(node), conf_(conf) {}
  ~RobosenseDriver() { stop(); }
  bool init();

 private:
  void getPackets();
  void processMsopPackets();
  void processDifopPackets();

 private:
  std::shared_ptr<cyber::Node> node_ = nullptr;
  Config conf_;
  std::shared_ptr<cyber::Writer<RobosenseScan>> scan_writer_ = nullptr;
  std::shared_ptr<cyber::Writer<PointCloud>> pointcloud_writer_ = nullptr;

 private:
  void prepareLidarScanMsg(std::shared_ptr<RobosenseScan> msg);
  void preparePointsMsg(std::shared_ptr<PointCloud> msg);
  void stop() {
    msop_pkt_queue_.clear();
    difop_pkt_queue_.clear();
    if (thread_flag_ == true) {
      thread_flag_ = false;
      lidar_thread_ptr_->join();
    }
  }

 private:
  Queue<LidarPacketMsg> msop_pkt_queue_;
  Queue<LidarPacketMsg> difop_pkt_queue_;
  bool thread_flag_;
  std::shared_ptr<std::thread> lidar_thread_ptr_;
  std::shared_ptr<DecoderBase<PointXYZIT>> lidar_decoder_ptr_;
  std::shared_ptr<Input> lidar_input_ptr_;
  uint32_t scan_seq_;
  uint32_t points_seq_;
  std::shared_ptr<PointCloud> point_cloud_ptr_;
  std::shared_ptr<RobosenseScan> scan_ptr_;
  std::shared_ptr<cyber::base::ThreadPool> thread_pool_ptr_;

  // gwh added
  uint32_t points_seq1_;
  bool publish_full_pointcloud_msg_;
  std::shared_ptr<DecoderBase<PointXYZITR_RS>> lidar_decoder_ptr_withFullInfo_;
  std::shared_ptr<cyber::Writer<PointCloud_withFullInfo>> pointcloud_writer_withFullInfo_ = nullptr;
  std::shared_ptr<PointCloud_withFullInfo> point_cloud_ptr_withFullInfo_;

  void preparePointsMsg(std::shared_ptr<PointCloud_withFullInfo> msg);
  // added end
};
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
