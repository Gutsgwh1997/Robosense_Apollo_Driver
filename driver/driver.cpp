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
#include "modules/drivers/robosense/driver/driver.h"

namespace apollo {
namespace drivers {
namespace robosense {
using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::cyber::base::ThreadPool;
using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::robosense::RobosenseScan;
bool RobosenseDriver::init() {
  if (node_ == nullptr) {
    AERROR << "node is nullptr";
    return false;
  }
  scan_writer_ = node_->CreateWriter<RobosenseScan>(conf_.scan_channel());
  pointcloud_writer_ =
      node_->CreateWriter<PointCloud>(conf_.pointcloud_channel());
  if (scan_writer_ == nullptr) {
    AERROR << "writer:" << conf_.scan_channel()
           << " create error, check cyber is inited.";
    return false;
  }
  uint32_t split_frame_mode;
  split_frame_mode = conf_.split_frame_mode();
  publish_full_pointcloud_msg_ = conf_.publish_full_pointcloud_msg();
  RSDriverParam param;
  param.decoder_param.min_distance = conf_.min_distance();
  param.decoder_param.max_distance = conf_.max_distance();
  param.decoder_param.start_angle = conf_.start_angle();
  param.decoder_param.end_angle = conf_.end_angle();
  param.decoder_param.use_lidar_clock = conf_.use_lidar_clock();
  param.decoder_param.num_pkts_split = conf_.num_pkts_split();
  param.decoder_param.cut_angle = conf_.cut_angle();
  param.angle_path = conf_.angle_path();
  LidarType lidar_type = param.strToLidarType(conf_.model());
  param.lidar_type = lidar_type;
  param.decoder_param.split_frame_mode = SplitFrameMode(split_frame_mode);

  if (conf_.model() != "RS16" && conf_.model() != "RS32" &&
      conf_.model() != "RS128" && conf_.model() != "RSBP" &&
      conf_.model() != "RS80") {
    AERROR << "Wrong input LiDAR Type!";
    return false;
  }

  if(publish_full_pointcloud_msg_){
    pointcloud_writer_withFullInfo_ = 
        node_->CreateWriter<PointCloud_withFullInfo>(conf_.pointcloud_channel() + "_withFullInfo");

    lidar_decoder_ptr_withFullInfo_ =
        DecoderFactory<PointXYZITR_RS>::createDecoder(lidar_type, param);
  }
  point_cloud_ptr_withFullInfo_ = std::make_shared<PointCloud_withFullInfo>();

  lidar_decoder_ptr_ =
      DecoderFactory<PointXYZIT>::createDecoder(lidar_type, param);
  lidar_input_ptr_ =
      std::make_shared<Input>(conf_.msop_port(), conf_.difop_port());
  point_cloud_ptr_ = std::make_shared<PointCloud>();
  scan_ptr_ = std::make_shared<RobosenseScan>();
  thread_pool_ptr_ = std::make_shared<ThreadPool>(4);
  points_seq_ = 0;
  points_seq1_ = 0;
  scan_seq_ = 0;
  thread_flag_ = true;
  lidar_thread_ptr_ = std::make_shared<std::thread>([this] { getPackets(); });
  return true;
}

void RobosenseDriver::getPackets() {
  while (thread_flag_) {
    LidarPacketMsg pkt_msg;
    InputState ret = lidar_input_ptr_->getPacket(pkt_msg.packet.data(), 100);
    if (ret == INPUT_MSOP) {
      msop_pkt_queue_.push(pkt_msg);
      if (msop_pkt_queue_.is_task_finished.load()) {
        msop_pkt_queue_.is_task_finished.store(false);
        thread_pool_ptr_->Enqueue([this]() { processMsopPackets(); });
      }
    } else if (ret == INPUT_DIFOP) {
      difop_pkt_queue_.push(pkt_msg);
      if (difop_pkt_queue_.is_task_finished.load()) {
        difop_pkt_queue_.is_task_finished.store(false);
        thread_pool_ptr_->Enqueue([this]() { processDifopPackets(); });
      }
    } else if (ret == INPUT_ERROR || ret == INPUT_EXIT ||
               ret == INPUT_TIMEOUT) {
      AERROR << "ErrCode_LidarDriverInterrupt";
    }
  }
}

void RobosenseDriver::processMsopPackets() {
  while (msop_pkt_queue_.m_quque.size() > 0 && thread_flag_) {
    LidarPacketMsg pkt = msop_pkt_queue_.m_quque.front();
    std::shared_ptr<int> height_ptr = std::make_shared<int>();
    void *data_ptr = malloc(PKT_DATA_LENGTH);
    memcpy(data_ptr, pkt.packet.data(), PKT_DATA_LENGTH);
    scan_ptr_->add_firing_pkts()->set_data(data_ptr, PKT_DATA_LENGTH);
    free(data_ptr);
    msop_pkt_queue_.pop();
    std::shared_ptr<std::vector<PointXYZIT>> point_vec_ptr =
        std::make_shared<std::vector<PointXYZIT>>();

    std::shared_ptr<std::vector<PointXYZITR_RS>> point_vec_ptr_withFullInfo =
        std::make_shared<std::vector<PointXYZITR_RS>>();

    int ret = 0;
    if (thread_flag_) {
      ret = lidar_decoder_ptr_->processMsopPkt(pkt.packet.data(), point_vec_ptr,
                                               height_ptr);
      if (publish_full_pointcloud_msg_) {
        lidar_decoder_ptr_withFullInfo_->processMsopPkt(pkt.packet.data(), point_vec_ptr_withFullInfo, height_ptr);
      }
    }
    if (ret == RSDecoderResult::DECODE_OK ||
        ret == RSDecoderResult::FRAME_SPLIT) {
      for (auto iter : *point_vec_ptr) {
        if (std::isnan(iter.x()) || std::isnan(iter.y()) ||
            std::isnan(iter.z())) {
          continue;
        }
        PointXYZIT *point = point_cloud_ptr_->add_point();
        point->set_x(iter.x());
        point->set_y(iter.y());
        point->set_z(iter.z());
        point->set_intensity(iter.intensity());
        point->set_timestamp(iter.timestamp());
      }

      for (auto iter : *point_vec_ptr_withFullInfo) {
        PointXYZITR *point = point_cloud_ptr_withFullInfo_->add_point();
        point->set_x(iter.x);
        point->set_y(iter.y);
        point->set_z(iter.z);
        point->set_intensity(iter.intensity);
        point->set_timestamp(iter.timestamp);
        point->set_ring(iter.ring);
        if (std::isnan(iter.x) || std::isnan(iter.y) || std::isnan(iter.z)) {
          point->set_x(0.001);
          point->set_y(0.001);
          point->set_z(0.001);
          point->set_intensity(0);
        }
      }

      if (ret == RSDecoderResult::FRAME_SPLIT) {
        std::shared_ptr<PointCloud> raw_cloud = point_cloud_ptr_;
        raw_cloud->set_height(*height_ptr);
        preparePointsMsg(raw_cloud);

	std::shared_ptr<PointCloud_withFullInfo> raw_cloud_withFullInfo = point_cloud_ptr_withFullInfo_;
        if(publish_full_pointcloud_msg_){
          raw_cloud_withFullInfo->set_height(*height_ptr);
          preparePointsMsg(raw_cloud_withFullInfo);
        }

        if (conf_.use_lidar_clock()) {
          const auto timestamp =
              raw_cloud->point(static_cast<int>(raw_cloud->point_size()) - 1)
                  .timestamp();
          raw_cloud->set_measurement_time(static_cast<double>(timestamp) / 1e9);

          raw_cloud->mutable_header()->set_lidar_timestamp(timestamp);

          raw_cloud_withFullInfo->set_measurement_time(static_cast<double>(timestamp) / 1e9);
          raw_cloud_withFullInfo->mutable_header()->set_lidar_timestamp(timestamp);
        }
        if (raw_cloud->point_size() != 0) {
          pointcloud_writer_->Write(raw_cloud);
        }
        if (publish_full_pointcloud_msg_ && raw_cloud_withFullInfo->point_size() != 0){
          pointcloud_writer_withFullInfo_->Write(raw_cloud_withFullInfo);
        }
        std::shared_ptr<RobosenseScan> raw_scan = scan_ptr_;
        prepareLidarScanMsg(raw_scan);
        scan_writer_->Write(raw_scan);
        point_cloud_ptr_.reset(new PointCloud);
        scan_ptr_.reset(new RobosenseScan);
        if(publish_full_pointcloud_msg_){
          point_cloud_ptr_withFullInfo_.reset(new PointCloud_withFullInfo);
        }
      }
    }
  }
  msop_pkt_queue_.is_task_finished.store(true);
}

void RobosenseDriver::processDifopPackets() {
  while (difop_pkt_queue_.m_quque.size() > 0 && thread_flag_) {
    LidarPacketMsg pkt = difop_pkt_queue_.m_quque.front();
    difop_pkt_queue_.pop();
    lidar_decoder_ptr_->processDifopPkt(pkt.packet.data());
  }
  difop_pkt_queue_.is_task_finished.store(true);
}

void RobosenseDriver::prepareLidarScanMsg(std::shared_ptr<RobosenseScan> msg) {
  msg->mutable_header()->set_sequence_num(++scan_seq_);
  msg->mutable_header()->set_frame_id(conf_.frame_id());
  msg->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
  msg->set_model(conf_.model());
}
void RobosenseDriver::preparePointsMsg(std::shared_ptr<PointCloud> msg) {
  msg->set_width(msg->point_size() / msg->height());
  msg->set_is_dense(false);
  msg->mutable_header()->set_sequence_num(++points_seq_);
  msg->mutable_header()->set_frame_id(conf_.frame_id());
  msg->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
  msg->mutable_header()->set_lidar_timestamp(cyber::Time().Now().ToSecond() *
                                             kSecondToNanoFactor);
  const auto timestamp =
      msg->point(static_cast<int>(msg->point_size()) - 1).timestamp();
  msg->set_measurement_time(static_cast<double>(timestamp) / 1e9);
}

void RobosenseDriver::preparePointsMsg(std::shared_ptr<PointCloud_withFullInfo> msg) {
  msg->set_width(msg->point_size() / msg->height());
  msg->set_is_dense(false);
  msg->mutable_header()->set_sequence_num(++points_seq1_);
  msg->mutable_header()->set_frame_id(conf_.frame_id());
  msg->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
  msg->mutable_header()->set_lidar_timestamp(cyber::Time().Now().ToSecond() * kSecondToNanoFactor);
  const auto timestamp = msg->point(static_cast<int>(msg->point_size()) - 1).timestamp();
  msg->set_measurement_time(static_cast<double>(timestamp) / 1e9);
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo