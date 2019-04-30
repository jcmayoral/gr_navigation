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
            CURL *curl;
            CURLcode res;
            curl = curl_easy_init();
            // Response information.
            int httpCode(0);
            std::unique_ptr<std::string> httpData(new std::string());
            std::string url("https://www.yr.no/place/Mexico/Distrito_Federal/Mexico_City/forecast.xml");
            //std::string url("https://www.yr.no/place/Mexico/Distrito_Federal/Mexico_City/data.html");
            std::string buffer;

            if(curl) {
                  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
                  //just ipv4
                  curl_easy_setopt(curl, CURLOPT_IPRESOLVE, CURL_IPRESOLVE_V4);
                  //timeout n seconds
                  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10);
                  //redirect if necessary
                  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
                  /* Perform the request, res will get the return code */ 

                  res = curl_easy_perform(curl);
                  /* Check for errors */ 
                  if(res != CURLE_OK)
                        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                              curl_easy_strerror(res));

                  // Hook up data handling function.
                  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback);
                  // Hook up data container (will be passed as the last parameter to the
                  // callback handling function).  Can be any pointer type, since it will
                  // internally be passed as a void pointer.
                  curl_easy_setopt(curl, CURLOPT_WRITEDATA, httpData.get());
                  //curl_easy_setopt(curl, CURLOPT_WRITEDATA, buffer);

                  // Run our HTTP GET command, capture the HTTP response code, and clean up.
                  curl_easy_perform(curl);
                  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
                  /* always cleanup */ 
                  curl_easy_cleanup(curl);
            }
            else{
                  ROS_ERROR("Error");
            }

            if (httpCode == 200){
                  std::cout << "\nGot successful response from " << url << std::endl;
                  // Response looks good - done using Curl now.  Try to parse the results
                  // and print them out.
                  Json::Value jsonData;
                  Json::Reader jsonReader;


                  if (jsonReader.parse(*httpData.get(), jsonData)){
                        std::cout << "Successfully parsed JSON data" << std::endl;
                        std::cout << "\nJSON data received:" << std::endl;
                        std::cout << jsonData.toStyledString() << std::endl;
                        const std::string dateString(jsonData["date"].asString());
                        const std::size_t unixTimeMs(jsonData["milliseconds_since_epoch"].asUInt64());
                        const std::string timeString(jsonData["time"].asString());
                        std::cout << "Natively parsed:" << std::endl;
                        std::cout << "\tDate string: " << dateString << std::endl;
                        std::cout << "\tUnix timeMs: " << unixTimeMs << std::endl;
                        std::cout << "\tTime string: " << timeString << std::endl;
                        std::cout << std::endl;
                  }
                  else{
                        std::cout << "Could not parse HTTP data as JSON" << std::endl;
                        std::cout << "HTTP data was:\n" << *httpData.get() << std::endl;
                  }
            }
            else
{
                  std::cout << "Couldn't GET from " << url << " - exiting" << std::endl;
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