/*
 * \file
 *
 *  quick config
 *
 *
 * Copyright? (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 */

#include "ti_mmwave_rospkg/mmWaveCLI.h"
#include "ti_mmwave_rospkg/ParameterParser.h"

#include <ros/ros.h>
#include <cstdlib>
#include <fstream>
#include <stdio.h>
#include <regex>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mmWaveQuickConfig");
  ros::NodeHandle nh;
  ti_mmwave_rospkg::mmWaveCLI srv;
  if (argc != 2)
  {
    ROS_INFO("mmWaveQuickConfig: usage: mmWaveQuickConfig /file_directory/params.cfg");
    return 1;
  }
  else
  {
    ROS_INFO("mmWaveQuickConfig: Configuring mmWave device using config file: %s", argv[1]);
  }

  ros::ServiceClient client = nh.serviceClient<ti_mmwave_rospkg::mmWaveCLI>("/mmWaveCLI");
  std::ifstream myParams;
  ti_mmwave_rospkg::ParameterParser parser;
  // wait 10s for service to become available
  ros::service::waitForService("/mmWaveCLI", 10000);

  // wait 0.5 secs to avoid multi-sensor conflicts
  ros::Duration(0.5).sleep();

  myParams.open(argv[1]);

  if (myParams.is_open())
  {
    while (std::getline(myParams, srv.request.comm))
    {
      srv.request.comm.erase(std::remove(srv.request.comm.begin(), srv.request.comm.end(), '\r'), srv.request.comm.end());
      // Ignore comment lines (first non-space char is '%') or blank lines
      if (!(std::regex_match(srv.request.comm, std::regex("^\\s*%.*")) || std::regex_match(srv.request.comm, std::regex("^\\s*"))))
      {
        ROS_INFO("mmWaveQuickConfig: Sending command: '%s'", srv.request.comm.c_str());
        if (client.call(srv))
        {
          ROS_INFO_STREAM("**TK1: answer of srv: " << srv.response.resp);
          ROS_INFO("**TK2: answer of srv: '%s'", srv.response.resp.c_str());
          if (std::regex_search(srv.response.resp, std::regex("Done")))
          {
            ROS_INFO("mmWaveQuickConfig: Command successful (mmWave sensor responded with 'Done')");
            parser.ParamsParser(srv, nh);
          }
          else
          {
            ROS_ERROR("mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')");
            ROS_ERROR("mmWaveQuickConfig: Response: '%s'", srv.response.resp.c_str());
            return 1;
          }
        }
        else
        {
          ROS_ERROR("mmWaveQuickConfig: Failed to call service mmWaveCLI");
          ROS_ERROR("%s", srv.request.comm.c_str());
          return 1;
        }
      }
    }
    parser.CalParams(nh);
    myParams.close();
  }
  else
  {
    ROS_ERROR("mmWaveQuickConfig: Failed to open File %s", argv[1]);
    return 1;
  }
  ROS_INFO("mmWaveQuickConfig: mmWaveQuickConfig will now terminate.");
  ROS_INFO(" => Done configuring mmWave device using config file: %s", argv[1]);
  return 0;
}
