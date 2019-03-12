#include <gr_map_utils/tf_frame_publisher.h>

namespace gr_map_utils{
    TfFramePublisher::TfFramePublisher(){
        
        static_transformStamped_.header.frame_id = "world";
        static_transformStamped_.child_frame_id = "map";
        
        //TODO get from GPS?
        static_transformStamped_.transform.translation.x = 0;
        static_transformStamped_.transform.translation.y = 0;
        static_transformStamped_.transform.translation.z = 0;;
        
        
        tf2::Quaternion quat;
        quat.setRPY(0.0,0.0,0.0);//TODO get Orientation from GPS;
        static_transformStamped_.transform.rotation.x = quat.x();
        static_transformStamped_.transform.rotation.y = quat.y();
        static_transformStamped_.transform.rotation.z = quat.z();
        static_transformStamped_.transform.rotation.w = quat.w();
        }

    void TfFramePublisher::publishTfTransform(){
        static_transformStamped_.header.stamp = ros::Time::now();
        static_broadcaster_.sendTransform(static_transformStamped_);
    }
};