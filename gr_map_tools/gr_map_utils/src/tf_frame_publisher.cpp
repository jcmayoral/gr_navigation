#include <gr_map_utils/tf_frame_publisher.h>

namespace gr_map_utils{
    TfFramePublisher::TfFramePublisher(bool init, std::string origin_frame, std::string map_frame): origin_x_(0.0), origin_y_(0.0), initialize_{init}{
        static_transformStamped_.header.frame_id = origin_frame;
        static_transformStamped_.child_frame_id = map_frame;

        if (initialize_){

            //TODO get from GPS?
            ROS_INFO("Waiting for GPS topic");
            boost::shared_ptr<sensor_msgs::NavSatFix const> gps_msg;
            gps_msg =  ros::topic::waitForMessage<sensor_msgs::NavSatFix>("fix");
            ROS_INFO("GPS topic received");

            transformGPSToProjection(*gps_msg,origin_x_,origin_y_);

            static_transformStamped_.transform.translation.x = origin_x_;
            static_transformStamped_.transform.translation.y = origin_y_;
            static_transformStamped_.transform.translation.z = 0;;


            tf2::Quaternion quat;
            quat.setRPY(0.0,0.0,0.0);//TODO get Orientation from GPS;
            static_transformStamped_.transform.rotation.x = quat.x();
            static_transformStamped_.transform.rotation.y = quat.y();
            static_transformStamped_.transform.rotation.z = quat.z();
            static_transformStamped_.transform.rotation.w = quat.w();
        }
        else{
          ROS_INFO("skipping tf transform map world");
        }
    }

     TfFramePublisher::TfFramePublisher(YAML::Node config): origin_x_(0.0), origin_y_(0.0){
        initialize_ = (bool) config["enable_tf"].as<int>();

        if (config["type"].as<std::string>().compare("topic")==0){
            ROS_ERROR("TF TYPE TOPIC");
            static_transformStamped_.header.frame_id = config["origin_frame"].as<std::string>();
            static_transformStamped_.child_frame_id = config["output_frame"].as<std::string>();

            if (initialize_){

                //TODO get from GPS?
                ROS_INFO("Waiting for GPS topic");
                boost::shared_ptr<sensor_msgs::NavSatFix const> gps_msg;
                gps_msg =  ros::topic::waitForMessage<sensor_msgs::NavSatFix>("fix");
                ROS_INFO("GPS topic received");

                transformGPSToProjection(*gps_msg,origin_x_,origin_y_);

                static_transformStamped_.transform.translation.x = origin_x_;
                static_transformStamped_.transform.translation.y = origin_y_;
                static_transformStamped_.transform.translation.z = 0;;


                tf2::Quaternion quat;
                quat.setRPY(0.0,0.0,0.0);//TODO get Orientation from GPS;
                static_transformStamped_.transform.rotation.x = quat.x();
                static_transformStamped_.transform.rotation.y = quat.y();
                static_transformStamped_.transform.rotation.z = quat.z();
                static_transformStamped_.transform.rotation.w = quat.w();
            }
            else{
            ROS_INFO("skipping tf transform map world");
            }
        }
        if (config["type"].as<std::string>().compare("landmark")==0){
            ROS_ERROR("TF TYPE LANDMARK");
            static_transformStamped_.header.frame_id = config["origin_frame"].as<std::string>();;
            static_transformStamped_.child_frame_id = config["output_frame"].as<std::string>();;

            if (initialize_){

                //TODO get from GPS?
                ROS_INFO("Waiting for GPS topic");
                sensor_msgs::NavSatFix gps_msg;
                gps_msg.latitude = config["latitude"].as<float>();
                gps_msg.longitude = config["longitude"].as<float>();
                ROS_INFO("GPS topic received");

                transformGPSToProjection(gps_msg,origin_x_,origin_y_);

                static_transformStamped_.transform.translation.x = origin_x_;
                static_transformStamped_.transform.translation.y = origin_y_;
                static_transformStamped_.transform.translation.z = 0;;


                tf2::Quaternion quat;
                quat.setRPY(0.0,0.0,0.0);//TODO get Orientation from GPS;
                static_transformStamped_.transform.rotation.x = quat.x();
                static_transformStamped_.transform.rotation.y = quat.y();
                static_transformStamped_.transform.rotation.z = quat.z();
                static_transformStamped_.transform.rotation.w = quat.w();
            }
            else{
            ROS_INFO("skipping tf transform map world");
            }
        }
    }

    void TfFramePublisher::publishTfTransform(){
        if (!initialize_){
          //std::cout << "skip publishing" << std::endl;
          return;
        }
        static_transformStamped_.header.stamp = ros::Time::now();
        static_broadcaster_.sendTransform(static_transformStamped_);
    }

    void TfFramePublisher::transformGPSToProjection(sensor_msgs::NavSatFix gps_msg, float& x, float& y){

        geographic_msgs::GeoPoint from;
        from.latitude = gps_msg.latitude;
        from.longitude = gps_msg.longitude;

        geodesy::UTMPoint to;
        geodesy::fromMsg(from, to);

        x = to.easting;
        y = to.northing;

    }

};
