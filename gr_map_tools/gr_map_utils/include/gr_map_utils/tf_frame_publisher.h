#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <math.h>
#include <geodesy/utm.h>
//YAML
#include <yaml-cpp/yaml.h>


namespace gr_map_utils{
    class TfFramePublisher{
        public:
            TfFramePublisher(YAML::Node config);
            TfFramePublisher(bool init=true, std::string origin_frame="world", std::string map_frame="map");
            ~TfFramePublisher();
            void publishTfTransform();
            void transformGPSToProjection(sensor_msgs::NavSatFix gps_msg, float& x, float& y);

            void getTf(float& x, float &y){
                x = origin_x_;
                y = origin_y_;
            }

            float getEuclideanDistanceToOrigin(float x, float y){
                return sqrt(pow(x,2) + pow(y,2));
            }

            float getOriginX(){
                return origin_x_;
            }

            float getOriginY(){
                return origin_y_;
            }

        private:
            tf2_ros::StaticTransformBroadcaster static_broadcaster_;
            geometry_msgs::TransformStamped static_transformStamped_;
            float origin_x_;
            float origin_y_;
            bool initialize_;
    };
}
