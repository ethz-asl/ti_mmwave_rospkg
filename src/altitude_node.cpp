//
// Created by mpantic on 31.08.21.
//
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Range.h>
#include <ti_mmwave_rospkg/altitude_node.h>

AltitudeNode::AltitudeNode(ros::NodeHandle nh) {
  pub_range_ = nh.advertise<sensor_msgs::Range>("range", 10);

  sub_radar_packet_ = nh.subscribe("/ti_mmwave/radar_scan", 1,
                                   &AltitudeNode::radarScanCallback, this);
}

void AltitudeNode::radarScanCallback(
    const ti_mmwave_rospkg::RadarScan::ConstPtr &msg) {

  updateFilter(*msg);


  sensor_msgs::Range range;
  range.header.stamp = msg->header.stamp;
  range.range = mean_;
  pub_range_.publish(range);

}

void AltitudeNode::updateFilter(const ti_mmwave_rospkg::RadarScan &scan) {
  if (var_ == 0) {
    // first startup init
    last_timestamp_ = scan.header.stamp;
    mean_ = scan.x;
    var_ = std::pow(measurement_noise_, 2) * 5;
    ROS_INFO_STREAM("Init to " << mean_ << " / " << var_);
    return;
  }

  // do maha outlier distance
  double bearing_y = atan2(scan.y, scan.x);
  double bearing_z = atan2(scan.z, scan.x);

  if (std::max(bearing_y, bearing_z) > max_bearing_ ||
      scan.intensity < min_intensity_) {

    return; // ignore
  }
  // fake proces model before update
  double dt = (scan.header.stamp - last_timestamp_).toSec();
  var_ += std::pow(process_noise_ * dt, 2);

  double y = scan.x - mean_;
  double S = var_ + std::pow(measurement_noise_, 2);
  double K = var_ / S;
  mean_ += K * y;
  var_ = (1.0 - K) * var_;

  //ROS_WARN_STREAM(mean_ << "\t" << var_);
  last_timestamp_ = scan.header.stamp;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ti_mmwave_altitude_node");
  ros::NodeHandle nh;

  AltitudeNode node(nh);

  ros::spin();
}
