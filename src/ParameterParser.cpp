/*
 * \file
 *
 *  parameter parser
 *
 *
 * Copyright? (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 */

#include "ti_mmwave_rospkg/ParameterParser.h"
#include <string>
#include <vector>

namespace ti_mmwave_rospkg
{

  PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::ParameterParser, nodelet::Nodelet);

  ParameterParser::ParameterParser() {}

  void ParameterParser::onInit() {}

  void ParameterParser::ParamsParser(ti_mmwave_rospkg::mmWaveCLI &srv, ros::NodeHandle &nh)
  {
    // ROS_ERROR("%s",srv.request.comm.c_str());
    // ROS_ERROR("%s",srv.response.resp.c_str());
    std::vector<std::string> v;
    std::string s = srv.request.comm.c_str();
    std::istringstream ss(s);
    std::string token;
    std::string req;
    int i = 0;
    while (std::getline(ss, token, ' '))
    {
      v.push_back(token);
      if (i > 0)
      {
        if (!req.compare("profileCfg"))
        {
          switch (i)
          {
          case 2:
            nh.setParam("startFreq", std::stof(token));
            break;
          case 3:
            nh.setParam("idleTime", std::stof(token));
            break;
          case 4:
            nh.setParam("adcStartTime", std::stof(token));
            break;
          case 5:
            nh.setParam("rampEndTime", std::stof(token));
            break;
          case 8:
            nh.setParam("freqSlopeConst", std::stof(token));
            break;
          case 10:
            nh.setParam("numAdcSamples", std::stoi(token));
            break;
          case 11:
            nh.setParam("digOutSampleRate", std::stof(token));
            break;
          case 14:
            nh.setParam("rxGain", std::stof(token));
            break;
          }
        }
        else if (!req.compare("frameCfg"))
        {
          switch (i)
          {
          case 1:
            nh.setParam("chirpStartIdx", std::stoi(token));
            break;
          case 2:
            nh.setParam("chirpEndIdx", std::stoi(token));
            break;
          case 3:
            nh.setParam("numLoops", std::stoi(token));
            break;
          case 4:
            nh.setParam("numFrames", std::stoi(token));
            break;
          case 5:
            nh.setParam("framePeriodicity", std::stof(token));
            break;
          }
        }
      }
      else
        req = token;
      i++;
    }
  }

  void ParameterParser::CalParams(ros::NodeHandle &nh)
  {
    float c0 = 299792458;
    int chirpStartIdx;
    int chirpEndIdx;
    int numLoops;
    float framePeriodicity;
    float startFreq;
    float idleTime;
    float adcStartTime;
    float rampEndTime;
    float digOutSampleRate;
    float freqSlopeConst;
    float numAdcSamples;

    ROS_WARN_STREAM("***in datahandler: startFreq " << startFreq);
    nh.getParam("startFreq", startFreq);
    nh.getParam("idleTime", idleTime);
    nh.getParam("adcStartTime", adcStartTime);
    nh.getParam("rampEndTime", rampEndTime);
    nh.getParam("digOutSampleRate", digOutSampleRate);
    nh.getParam("freqSlopeConst", freqSlopeConst);
    nh.getParam("numAdcSamples", numAdcSamples);

    nh.getParam("chirpStartIdx", chirpStartIdx);
    nh.getParam("chirpEndIdx", chirpEndIdx);
    nh.getParam("numLoops", numLoops);
    nh.getParam("framePeriodicity", framePeriodicity);
    ROS_WARN_STREAM("***in datahandler: startFreq " << startFreq);

    int ntx = chirpEndIdx - chirpStartIdx + 1;
    int nd = numLoops;
    int nr = numAdcSamples;
    float tfr = framePeriodicity * 1e-3;
    float fs = digOutSampleRate * 1e3;
    float kf = freqSlopeConst * 1e12;
    float adc_duration = nr / fs;
    float BW = adc_duration * kf;
    float PRI = (idleTime + rampEndTime) * 1e-6;
    float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2);

    float vrange = c0 / (2 * BW);
    float max_range = nr * vrange;
    float max_vel = c0 / (2 * fc * PRI) / ntx;
    float vvel = max_vel / nd;

    ROS_WARN_STREAM("***in datahandler: num_TX " << ntx);
    nh.setParam("num_TX", ntx);
    nh.setParam("f_s", fs);
    nh.setParam("f_c", fc);
    nh.setParam("BW", BW);
    nh.setParam("PRI", PRI);
    nh.setParam("t_fr", tfr);
    nh.setParam("max_range", max_range);
    nh.setParam("range_resolution", vrange);
    nh.setParam("max_doppler_vel", max_vel);
    nh.setParam("doppler_vel_resolution", vvel);

    int asdf;
    nh.getParam("num_TX", asdf);
    ROS_WARN_STREAM("***in datahandler: num_TX " << asdf);
  }

}  // namespace ti_mmwave_rospkg
