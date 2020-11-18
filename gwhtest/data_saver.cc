#include <fstream>
#include <sstream>
#include <string>
#include <assert.h>
#include <cmath>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "cyber/cyber.h"
#include "boost/filesystem.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/robosense/proto/pointcloud_withfullinfo.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/robosense/gwhtest/help.hpp"

using namespace std;

using apollo::cyber::Node;
using apollo::cyber::Reader;
using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::robosense::PointCloud_withFullInfo;
using apollo::drivers::Image;
using apollo::drivers::gnss::Imu;

void removeAllFilesInUnemptyDir(const string& dir);

// 全局变量设定区
string pointCloudSaveDir_ = "/apollo/data/gwh/rs32_pointCloud/";
string imuSaveDir_ = "/apollo/data/gwh/imu/";
string imgSaveDir_ = "/apollo/data/gwh/img/";
ofstream lidarIndex_;
ofstream imuIndex_;
ofstream imgIndex_;

/**
 * @brief 点云消息的回调函数
 *
 * @param msg 自定义消息（robosense/proto下）
 */
void LidarMsgCallBack(const std::shared_ptr<PointCloud_withFullInfo>& msg) {
  static size_t count = 0;
  pcl::PointCloud<PointXYZIRT>::Ptr pcl_ptr(new pcl::PointCloud<PointXYZIRT>);
  pcl_ptr->width = msg->width();
  pcl_ptr->height = msg->height();
  pcl_ptr->is_dense = false;
  int point_size = pcl_ptr->width * pcl_ptr->height;
  assert(point_size <= msg->point_size());
  pcl_ptr->points.resize(point_size);

  for (size_t i = 0; i < pcl_ptr->points.size(); ++i) {
    const auto& point = msg->point(static_cast<int>(i));
    pcl_ptr->points[i].x = point.x();
    pcl_ptr->points[i].y = point.y();
    pcl_ptr->points[i].z = point.z();
    pcl_ptr->points[i].intensity = point.intensity();
    pcl_ptr->points[i].ring = point.ring();
    pcl_ptr->points[i].timestamp = point.timestamp();

    if (point.x() == 0.001 && point.y() == 0.001 && point.z() == 0.001 && point.intensity() == 0) {
        pcl_ptr->points[i].x = NAN;
        pcl_ptr->points[i].y = NAN;
        pcl_ptr->points[i].z = NAN;
    }
    // 测试每个点的时间戳，具体点的时间戳是一个packet一变
    // if (count == 10) {
    //   lidarIndex_ << point.timestamp() << " ";
    //   if ((i + 1) % 32 == 0) lidarIndex_ << std::endl;
    // }

    // 测试激光点的垂直角度
    // if (count == 10) {
    //   double c = sqrt(point.x() * point.x() + point.y() * point.y());
    //   double angle = atan2(point.z(), c) * 180 / 3.14159;
    //   lidarIndex_ << setiosflags(ios::right) << setprecision(4) << setw(8) << angle << " ";
    //   if ((i + 1) % 32 == 0) lidarIndex_ << std::endl;
    // }

    // 测试激光点的ring
    // if (count == 10) {
    //   lidarIndex_ << pcl_ptr->points[i].ring<<" ";
    //   if ((i + 1) % 32 == 0) lidarIndex_ << std::endl;
    // }
  }

  // 测试一帧点云的时间戳
  // if(count == 10){
  //     lidarIndex_ << msg->measurement_time()<<" ";                                      // double类型，单位是s（下一个/1e9）
  //     lidarIndex_ << msg->mutable_header()->lidar_timestamp()<<" "                      // uint64类型的，单位是ns
  //     lidarIndex_ << msg->point(static_cast<int>(point_size - 1)).timestamp()<<endl;    // 由于截断数据，时间戳偏移
  // }
  // uint64_t point_cloud_timestamp = msg->mutable_header()->lidar_timestamp();
  // 硬件返回的时间
  // double point_cloud_timestamp = msg->measurement_time();
  // 消息发布的时间
  double measurement_time2 = msg->mutable_header()->timestamp_sec();

  std::stringstream file_path;
  file_path << pointCloudSaveDir_ << std::setfill('0') << std::setw(10) << count << ".pcd";
  lidarIndex_ << file_path.str() << " " << setiosflags(ios::fixed) << setprecision(9) << measurement_time2<<endl;
  // 测试点云保存需要的时间
  // struct timeval before, after;
  // gettimeofday(&before, NULL);
  // pcl::io::savePCDFileASCII(file_path.str(), *pcl_ptr);
  pcl::io::savePCDFileBinaryCompressed(file_path.str(), *pcl_ptr);
  // gettimeofday(&after, NULL);
  // double time_consume = ((double)(after.tv_sec - before.tv_sec) + (double)(after.tv_usec - before.tv_usec) / 1e6) * 1000;

  BOLD_YELLOW << "Saved " << setfill('0') << setw(6) << ++count<<" lidar frames, timestamp: ";
  BOLD_YELLOW << setiosflags(ios::fixed) << setprecision(3) << measurement_time2 << ENDL;
  // std::cout << "Point cloud width: " << msg->width() << " height: " << msg->height() << " size: " << msg->point_size() << std::endl;
  return;
}

/**
 * @brief Image消息回调函数
 */
void ImgMsgCallBack(const std::shared_ptr<Image>& msg){
    static size_t count = 0;
    // 硬件返回的测量时间
    // double measurement_time = msg->measurement_time();
    // 消息发布的时间
    double measurement_time2 = msg->mutable_header()->timestamp_sec();
    // 这个时间戳始终是0
    // uint64_t measurement_time = msg->mutable_header()->camera_timestamp();

    int height = msg->height();
    int width = msg->width();
    // int step = msg->step();
    // int bytes = step * height;
    string encoding = msg->encoding();
    auto& data = msg->data();
    void* pdata = const_cast<char*>(&data[0]);
    cv::Mat rgb_img(height, width, CV_8UC3, pdata);
    cv::Mat bgr_img;
    cv::cvtColor(rgb_img, bgr_img,cv::COLOR_RGB2BGR);

    std::stringstream file_path;
    file_path << imgSaveDir_ << std::setfill('0') << std::setw(10) << count << ".jpg";
    imgIndex_ << file_path.str() << " " << setiosflags(ios::fixed) << setprecision(9) << measurement_time2 << endl;
    cv::imwrite(file_path.str(), bgr_img);

    // BOLD_GREEN << "Saved "<< setfill('0') << setw(6)<< ++count <<" images, "<< "encoding: "<< encoding ;
    BOLD_GREEN << "Saved "<< setfill('0') << setw(6)<< ++count <<" image frames";
    BOLD_GREEN <<", timestamps: "<< setiosflags(ios::fixed) << setprecision(3) << measurement_time2 << ENDL;
}

/**
 * @brief IMU消息回调函数
 */
void ImuMsgCallBack(const std::shared_ptr<Imu>& msg){
  static size_t count = 0;
  // 硬件返回的测量时间
  // double measurement_time = msg->measurement_time();
  // 消息发布的时间
  double measurement_time2 = msg->mutable_header()->timestamp_sec();

  // double ax = msg->linear_acceleration().x();
  // double ay = msg->linear_acceleration().y();
  // double az = msg->linear_acceleration().z();
  // double wx = msg->angular_velocity().x();
  // double wy = msg->angular_velocity().y();
  // double wz = msg->angular_velocity().z();
  // imuIndex_ << setiosflags(ios::fixed) << setprecision(9) << measurement_time2 <<" ";
  // imuIndex_ << ax << " " << ay << " " << az << " ";
  // imuIndex_ << wx << " " << wy << " " << wz << endl;
  ImuData imumsg;
  imumsg.measurement_time = msg->mutable_header()->timestamp_sec();
  imumsg.ax = msg->linear_acceleration().x();
  imumsg.ay = msg->linear_acceleration().y();
  imumsg.az = msg->linear_acceleration().z();
  imumsg.wx = msg->angular_velocity().x();
  imumsg.wy = msg->angular_velocity().y();
  imumsg.wz = msg->angular_velocity().z();
  imuIndex_ << imumsg;

  BOLD_WHITE << "Saved "<< setfill('0') << setw(6)<< ++count <<" imu frames, ";
  BOLD_WHITE << "timestamp: " << setiosflags(ios::fixed) << setprecision(3) << measurement_time2 << ENDL;
}

int main(int argc, char* argv[]) {
    apollo::cyber::Init(argv[0]);

    removeAllFilesInUnemptyDir(pointCloudSaveDir_);
    removeAllFilesInUnemptyDir(imuSaveDir_);
    removeAllFilesInUnemptyDir(imgSaveDir_);

    string lidarFile = pointCloudSaveDir_ + "lidar.txt";
    string imuFile = imuSaveDir_ + "imu.txt";
    string imgFile = imgSaveDir_ + "img.txt";

    lidarIndex_.open(lidarFile);
    imuIndex_.open(imuFile);
    imgIndex_.open(imgFile);

    auto listener_node = apollo::cyber::CreateNode("data_saver");

    auto lidar_listener = listener_node->CreateReader<PointCloud_withFullInfo>("/apollo/sensor/rs32/PointCloud2_withFullInfo", LidarMsgCallBack);
    auto img_listener = listener_node->CreateReader<Image>("/apollo/sensor/camera/front_6mm/image", ImgMsgCallBack);
    auto imu_listener = listener_node->CreateReader<Imu>("/apollo/sensor/gnss/imu", ImuMsgCallBack);
    apollo::cyber::WaitForShutdown();

    lidarIndex_.close();
    imuIndex_.close();
    imgIndex_.close();

    return 0;
}





/**
 * @brief 移除非空文件夹下的全部文件
 *
 * @param dir 文件夹的路径
 */
void removeAllFilesInUnemptyDir(const string& dir) {
    string file_path = dir;
    boost::filesystem::path tmp_path(file_path.c_str());

    if(!boost::filesystem::exists(tmp_path)){
        boost::filesystem::create_directories(tmp_path);
        return;
    }

    if (!boost::filesystem::is_empty(tmp_path)) {
        boost::filesystem::directory_iterator diter(tmp_path);
        boost::filesystem::directory_iterator diter_end;

        for (; diter != diter_end; ++diter) {
            if (!boost::filesystem::is_regular_file(diter->status())) continue;
            boost::filesystem::remove(diter->path());
        }
    }

    return;
}
