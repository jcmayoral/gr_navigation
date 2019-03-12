#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

namespace gr_map_utils{
    class TfFramePublisher{
        public:
            TfFramePublisher();
            ~TfFramePublisher();
            void publishTfTransform();
        private:
            tf2_ros::StaticTransformBroadcaster static_broadcaster_;
            geometry_msgs::TransformStamped static_transformStamped_;
    };
}