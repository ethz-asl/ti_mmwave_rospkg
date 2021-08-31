//
// Created by mpantic on 31.08.21.
//

#ifndef TI_MMWAVE_ROSPKG_ALTITUDE_NODE_H
#define TI_MMWAVE_ROSPKG_ALTITUDE_NODE_H
#include <queue>
#include <ros/ros.h>
#include <ti_mmwave_rospkg/RadarScan.h>

class AltitudeNode {

public:
  AltitudeNode(ros::NodeHandle nh);

  void radarScanCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr &msg);

  void updateQueue(const ti_mmwave_rospkg::RadarScan &scan);

  void updateFilter(const ti_mmwave_rospkg::RadarScan &scan);

  std::pair<double, double> getRangeAndVel();

private:
  ros::Publisher pub_range_;
  ros::Subscriber sub_radar_packet_;

  double min_intensity_{15.0};      // [? whatever]
  double max_bearing_{15.0/180*M_PI};        // [rad]

  double mean_{0.0}, var_{0.0};
  double measurement_noise_{0.5};
  double process_noise_{1.0}; // [m] (gets scaled by time)

  ros::Time last_timestamp_;
};

#endif // TI_MMWAVE_ROSPKG_ALTITUDE_NODE_H
