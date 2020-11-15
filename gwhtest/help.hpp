#pragma once

#include <cstdint>
#include <iostream>
#include <iomanip>
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

// color cout
#ifndef GREEN
#define GREEN       (std::cout << "\033[32m")
#endif
#ifndef BOLD_GREEN
#define BOLD_GREEN  (std::cout << "\033[1m\033[32m")
#endif
#ifndef BOLD_YELLOW
#define BOLD_YELLOW (std::cout << "\033[1m\033[33m")
#endif
#ifndef BOLD_RED
#define BOLD_RED    (std::cout << "\033[1m\033[31m")
#endif
#ifndef BOLD_BLUE
#define BOLD_BLUE   (std::cout << "\033[1m\033[36m")
#endif
#ifndef BOLD_WHITE
#define BOLD_WHITE  (std::cout << "\033[1m\033[37m")
#endif
#ifndef ENDL
#define ENDL        "\033[0m" << std::endl
#endif
// color cout end

using namespace std;

// 自定义Pcl中的点类型
struct PointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    ( std::uint16_t, ring, ring)(double, timestamp, timestamp))

struct ImuData {
  double measurement_time;    // 硬件返回的时间(s)
  double ax;                  // x轴向的加速度分量
  double ay;                  // y轴向的加速度分量
  double az;                  // z轴向的加速度分量
  double wx;                  // x轴向的角速度分量
  double wy;                  // y轴向的角速度分量
  double wz;                  // z轴向的角速度分量
};
ostream& operator<<(ostream& out, const ImuData& imumsg){
  out << setiosflags(ios::fixed) << setprecision(9) << imumsg.measurement_time << " ";
  out << imumsg.ax << " " << imumsg.ay << " " << imumsg.az << " ";
  out << imumsg.wx << " " << imumsg.wy << " " << imumsg.wz << endl;
  return out;
}
istream& operator>>(istream& in, ImuData& imumsg){
  in >> imumsg.measurement_time;
  in >> imumsg.ax >> imumsg.ay >> imumsg.az;
  in >> imumsg.wx >> imumsg.wy >> imumsg.wz;
  return in;
}
