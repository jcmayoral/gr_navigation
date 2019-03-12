#include <gr_map_utils/tf_frame_publisher.h>

namespace gr_map_utils{
    TfFramePublisher::TfFramePublisher(){
        
        static_transformStamped_.header.frame_id = "world";
        static_transformStamped_.child_frame_id = "map";
        
        //TODO get from GPS?
        ROS_INFO("Waiting for GPS topic");
        boost::shared_ptr<sensor_msgs::NavSatFix const> gps_msg;
        gps_msg =  ros::topic::waitForMessage<sensor_msgs::NavSatFix>("android/fix");
        ROS_INFO("GPS topic received");
        float x = 0.0;
        float y = 0.0;
        transformGPSToProjection(*gps_msg,x,y);
        std::cout << "X " << x << std::endl;
        std::cout << "Y " << y << std::endl;

        static_transformStamped_.transform.translation.x = x;
        static_transformStamped_.transform.translation.y = y;
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

    void TfFramePublisher::transformGPSToProjection(sensor_msgs::NavSatFix gps_msg, float& x, float& y){
        float R = 6371000;
        float lat = gps_msg.latitude* M_PI /180;
        float lon = gps_msg.longitude * M_PI /180;
        x = R*cos(lon) * cos(lat);
        y = R*sin(lon) * cos(lat);
        float z = R*sin(lat);
        std::cout << "Z " << z << std::endl;
    }

};