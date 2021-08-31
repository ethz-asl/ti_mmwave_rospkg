//
// Created by mpantic on 31.08.21.
//

#ifndef TI_MMWAVE_ROSPKG_ALTITUDE_NODE_H
#define TI_MMWAVE_ROSPKG_ALTITUDE_NODE_H
#include <ros/ros.h>
#include <queue>
#include <ti_mmwave_rospkg/RadarScan.h>

class AltitudeNode {

public:
  AltitudeNode(ros::NodeHandle nh);

  void radarScanCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr &msg);

  void updateQueue(const ti_mmwave_rospkg::RadarScan& scan);

  std::pair<double, double> getRangeAndVel();

private:
  ros::Publisher pub_range_;
  ros::Subscriber sub_radar_packet_;

  std::deque<ti_mmwave_rospkg::RadarScan> scan_queue_;

  double min_intensity_{10.0};  // [? whatever]
  double max_bearing_{25.0};    // [deg]
  double sliding_window_time_{0.5}; //[s]

  double mean, var;
  double measurement_noise = 1.0;
};

#endif // TI_MMWAVE_ROSPKG_ALTITUDE_NODE_H
