#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <math.h>
#include <geodesy/utm.h>

namespace gr_map_utils{
    class TfFramePublisher{
        public:
            TfFramePublisher();
            ~TfFramePublisher();
            void publishTfTransform();
            void transformGPSToProjection(sensor_msgs::NavSatFix gps_msg, float& x, float& y);
        private:
            tf2_ros::StaticTransformBroadcaster static_broadcaster_;
            geometry_msgs::TransformStamped static_transformStamped_;
    };
}