#ifndef WEATHER_POLICY_H
#define WEATHER_POLICY_H

#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>

namespace gr_safety_policies
{

  class WeatherPolicy : public fault_core::FaultDetector
  {
    public:

      WeatherPolicy();
      ~WeatherPolicy();

      void instantiateServices(ros::NodeHandle nh);
      void initialize(int sensor_number);
      bool detectFault();
      void isolateFault();
      void diagnoseFault();
      fault_core::FaultTopology getFault();
  };


};

#endif
