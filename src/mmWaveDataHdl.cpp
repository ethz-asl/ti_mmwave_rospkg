/*
 * \file
 *
 *  data handler
 *
 *
 * Copyright? (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 */

#include "ti_mmwave_rospkg/mmWaveDataHdl.h"
#include "ti_mmwave_rospkg/DataHandlerClass.h"
#include <string>

namespace ti_mmwave_rospkg
{

  PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveDataHdl, nodelet::Nodelet);

  mmWaveDataHdl::mmWaveDataHdl() {}

  void mmWaveDataHdl::onInit()
  {
    ros::NodeHandle private_nh("~");

    std::string mySerialPort;
    std::string myFrameID;
    int myBaudRate;
    int myMaxAllowedElevationAngleDeg;
    int myMaxAllowedAzimuthAngleDeg;

    private_nh.getParam("data_port", mySerialPort);

    private_nh.getParam("data_rate", myBaudRate);

    private_nh.getParam("frame_id", myFrameID);

    if (!(private_nh.getParam("max_allowed_elevation_angle_deg", myMaxAllowedElevationAngleDeg)))
    {
      myMaxAllowedElevationAngleDeg = 90;  // Use max angle if none specified
    }

    if (!(private_nh.getParam("max_allowed_azimuth_angle_deg", myMaxAllowedAzimuthAngleDeg)))
    {
      myMaxAllowedAzimuthAngleDeg = 90;  // Use max angle if none specified
    }

    ROS_INFO("mmWaveDataHdl: data_port = %s", mySerialPort.c_str());
    ROS_INFO("mmWaveDataHdl: data_rate = %d", myBaudRate);
    ROS_INFO("mmWaveDataHdl: max_allowed_elevation_angle_deg = %d", myMaxAllowedElevationAngleDeg);
    ROS_INFO("mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d", myMaxAllowedAzimuthAngleDeg);

    DataUARTHandler DataHandler(&private_nh);
    DataHandler.setFrameID((char *)myFrameID.c_str());
    DataHandler.setUARTPort((char *)mySerialPort.c_str());
    DataHandler.setBaudRate(myBaudRate);
    DataHandler.setMaxAllowedElevationAngleDeg(myMaxAllowedElevationAngleDeg);
    DataHandler.setMaxAllowedAzimuthAngleDeg(myMaxAllowedAzimuthAngleDeg);
    DataHandler.start();

    NODELET_DEBUG("mmWaveDataHdl: Finished onInit function");
  }

}  // namespace ti_mmwave_rospkg
