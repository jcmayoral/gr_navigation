#include <mongodb_store/message_store.h>
#include <gr_map_utils/tf_frame_publisher.h>

namespace gr_map_utils{
    class MapConverterInterface{
        public:
            virtual bool storeMap() = 0;
            virtual bool getMap() = 0;
            virtual bool getMapFromTopic() = 0;
            virtual bool getMapFromService() = 0;
            virtual void transformMap() = 0;
            virtual void publishMaps() = 0;
            virtual ~MapConverterInterface(){}
        protected:
        	mongodb_store::MessageStoreProxy* message_store_;
            TfFramePublisher* gr_tf_publisher_;
    };
};
