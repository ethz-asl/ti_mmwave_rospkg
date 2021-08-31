//
// Created by mpantic on 31.08.21.
//
#include <sensor_msgs/Range.h>
#include <ti_mmwave_rospkg/altitude_node.h>

AltitudeNode::AltitudeNode(ros::NodeHandle nh) {
  pub_range_ = nh.advertise<sensor_msgs::Range>("range", 10);
  sub_radar_packet_ =
      nh.subscribe("/ti_mmwave/radar_scan", 1, &AltitudeNode::radarScanCallback, this);
}

void AltitudeNode::radarScanCallback(
    const ti_mmwave_rospkg::RadarScan::ConstPtr &msg) {

  updateQueue(*msg);
  auto range_and_vel = getRangeAndVel();

  sensor_msgs::Range range;
  range.header.stamp = msg->header.stamp;
  range.range = range_and_vel.first;

  pub_range_.publish(range);
}

void AltitudeNode::updateQueue(const ti_mmwave_rospkg::RadarScan &scan) {
  // add to queue
  scan_queue_.push_front(scan);

  while (scan_queue_.back().header.stamp +
             ros::Duration(sliding_window_time_) <
         scan.header.stamp) {
    scan_queue_.pop_back();
  }
}

std::pair<double, double> AltitudeNode::getRangeAndVel() {
  std::vector<double> ranges;

  for (int i = 0; i < scan_queue_.size(); i++) {

    if (scan_queue_[i].intensity > min_intensity_ &&
        scan_queue_[i].bearing < max_bearing_) {
      ranges.push_back(scan_queue_[i].x);
      std::cout << scan_queue_[i].x << "/" << scan_queue_[i].intensity << "/" << scan_queue_[i].bearing<< "/" << scan_queue_[i].velocity  << " ";
    }
  }
  std::cout << std::endl;

  ROS_INFO_STREAM(n << "\t " << range / n);
  return { range / n, 0.0 };
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ti_mmwave_altitude_node");
  ros::NodeHandle nh;

  AltitudeNode node(nh);

  ros::spin();

}