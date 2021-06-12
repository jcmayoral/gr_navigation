#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/foreach.hpp>
#include <mongodb_map_utils/tf_frame_publisher.h>
#include <boost/shared_ptr.hpp>
#include <std_srvs/Trigger.h>

namespace mongodb_map_utils{
    class MongoDBMapManager{
        public:
            MongoDBMapManager();
            ~MongoDBMapManager();
            void storeMessage();
            void getMapFrame();
            void publishStuff();
            bool update_frame_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

        private:
            mongodb_store::MessageStoreProxy* message_store_;
            ros::NodeHandle nh_;
            boost::shared_ptr<mongodb_map_utils::TfFramePublisher> tf_frame_publisher_;
            ros::ServiceServer update_server_;
    };
}