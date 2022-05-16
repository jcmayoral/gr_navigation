//#include <mongodb_store/message_store.h>
#include <warehouse_ros_mongo/database_connection.h>
#include <warehouse_ros_mongo/message_collection.h>
#include <gr_map_utils/tf_frame_publisher.h>
#include <navigation_msgs/TopologicalMap.h>
#include <mongodb_store/message_store.h>

# ifndef GR_MAP_UTILS_INTERFACE
# define GR_MAP_UTILS_INTERFACE


typedef warehouse_ros::MessageCollection<navigation_msgs::TopologicalMap> TopoMapCollection;
typedef warehouse_ros::MessageWithMetadata<navigation_msgs::TopologicalMap> TopoMapWithMetadata;
typedef TopoMapWithMetadata::ConstPtr TopoMapMetaPtr;

namespace gr_map_utils{
  typedef std::map<std::string, geometry_msgs::Pose> NodeMap;
  typedef std::pair<std::string, std::string> Edges;

  class MapConverterInterface{
    public:
      virtual bool storeMap() = 0;
      virtual bool getMapFromTopic() = 0;
      virtual bool getMapFromService() = 0;
      virtual bool getMapFromDatabase() = 0;

      bool getMap(){
        ROS_ERROR("GETMAP FUNCTION");
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
        ROS_ERROR("MAP NOT RETRIEVED");
        return false;
        };

        virtual void transformMap() = 0;
        virtual void publishMaps() = 0;
        virtual ~MapConverterInterface(){}

    protected:
      mongodb_store::MessageStoreProxy* message_store_;
      warehouse_ros_mongo::MongoDatabaseConnection mongo_connection_;
      TfFramePublisher* gr_tf_publisher_;
      bool is_map_received_;
  };
};

#endif
