#ifndef WEATHER_POLICY_H
#define WEATHER_POLICY_H

#include <ros/ros.h>
#include <string>
#include <safety_core/safe_policy.h>
#include <curl/curl.h>
#include <json/json.h>

//Based on
//https://gist.github.com/connormanning/41efa6075515019e499c

namespace gr_safety_policies
{
  std::size_t callback(const char* in, std::size_t size,
                       std::size_t num, std::string* out){
      const std::size_t totalBytes(size * num);
      out->append(in, totalBytes);
      return totalBytes;
  }

  class WeatherPolicy : public safety_core::SafePolicy
  {
    public:

      WeatherPolicy();
      ~WeatherPolicy();

      void instantiateServices(ros::NodeHandle nh);
      void initialize(int sensor_number);
      bool checkPolicy();
      void reportState();
      void suggestAction();
  };


};

#endif
