#include <mongodb_store/message_store.h>
#include <gr_map_utils/tf_frame_publisher.h>

namespace gr_map_utils{
    class MapConverterInterface{
        public:
            virtual bool storeMap() = 0;
            virtual bool getMapFromTopic() = 0;
            virtual bool getMapFromService() = 0;
            virtual bool getMapFromDatabase() = 0;

            bool getMap(){
                if (getMapFromDatabase()){
                    ROS_INFO("Retrieving map from database");
                    is_map_received_ = true;
                    return true;
                }

                if (getMapFromTopic()){
                    ROS_INFO("Retrieving map from topic");
                    is_map_received_ = true;
                    return true;
                }

                if(getMapFromService()){
                    ROS_INFO("Retrieving map from service");
                    is_map_received_ = true;
                    return true;
                }
                return false;
            };

            virtual void transformMap() = 0;
            virtual void publishMaps() = 0;
            virtual ~MapConverterInterface(){}
        protected:
        	mongodb_store::MessageStoreProxy* message_store_;
            TfFramePublisher* gr_tf_publisher_;
            bool is_map_received_;            
    };
};
