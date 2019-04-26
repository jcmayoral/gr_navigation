#include "gr_safety_policies/weather_policy.h"
#include <pluginlib/class_list_macros.h>
#include <curl/curl.h>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(gr_safety_policies, WeatherPolicy,
                        gr_safety_policies::WeatherPolicy,
                        fault_core::FaultDetector)

using namespace fault_core;
namespace gr_safety_policies
{
      WeatherPolicy::WeatherPolicy(){
            CURL *curl;
            CURLcode res;
            curl = curl_easy_init();
            if(curl) {
                  curl_easy_setopt(curl, CURLOPT_URL, "https://www.yr.no/place/Mexico/Distrito_Federal/Mexico_City/forecast.xml");
                  /* example.com is redirected, so we tell libcurl to follow redirection */ 
                  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
                  /* Perform the request, res will get the return code */ 

                  res = curl_easy_perform(curl);
                  /* Check for errors */ 
                  if(res != CURLE_OK)
                        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                              curl_easy_strerror(res));
 
            /* always cleanup */ 
            curl_easy_cleanup(curl);
            }
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