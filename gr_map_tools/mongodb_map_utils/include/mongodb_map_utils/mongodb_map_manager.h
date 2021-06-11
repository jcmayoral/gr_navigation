#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/foreach.hpp>
#include <mongodb_map_utils/tf_frame_publisher.h>
#include <boost/shared_ptr.hpp>

namespace mongodb_map_utils{
    class MongoDBMapManager{
        public:
            MongoDBMapManager();
            ~MongoDBMapManager();
            void storeMessage();
            void getMapFrame();
        private:
            mongodb_store::MessageStoreProxy* message_store_;
            ros::NodeHandle nh_;
            boost::shared_ptr<mongodb_map_utils::TfFramePublisher> tf_frame_publisher_;

    };
}