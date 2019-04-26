#include "gr_safety_policies/weather_policy.h"
#include <pluginlib/class_list_macros.h>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(gr_safety_policies, WeatherPolicy,
                        gr_safety_policies::WeatherPolicy,
                        fault_core::FaultDetector)

using namespace fault_core;
namespace gr_safety_policies
{
      WeatherPolicy::WeatherPolicy(){
      }

      WeatherPolicy::~WeatherPolicy(){
      }

      void WeatherPolicy::instantiateServices(ros::NodeHandle nh){
      }

      void WeatherPolicy::initialize(int sensor_number){
      }

      bool WeatherPolicy::detectFault(){
      }

      void WeatherPolicy::isolateFault(){
      }

      void WeatherPolicy::diagnoseFault(){
      }

      fault_core::FaultTopology WeatherPolicy::getFault(){
      }
}