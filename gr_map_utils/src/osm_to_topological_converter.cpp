#include <gr_map_utils/osm_to_topological_converter.h>

namespace gr_map_utils{

    Osm2TopologicalMap::Osm2TopologicalMap(ros::NodeHandle nh){
        gr_tf_publisher_ = new TfFramePublisher();
        message_store_ = new mongodb_store::MessageStoreProxy(nh);
        topological_map_pub_ = nh.advertise<strands_navigation_msgs::TopologicalMap>("topological_map", 1, true); 
    }

    Osm2TopologicalMap::~Osm2TopologicalMap(){

    }


    bool Osm2TopologicalMap::storeMap(){
        return false;
    }

    bool Osm2TopologicalMap::getMap(){
        return false;
    }


    void Osm2TopologicalMap::getMapFromTopic(){
        //        std::unique_lock<std::mutex> lk(mutex_);
        /*
        boost::shared_ptr<visualization_msgs::MarkerArray const> osm_map;
        osm_map =  ros::topic::waitForMessage<visualization_msgs::MarkerArray>("visualization_marker_array");
        if (osm_map != NULL){
            osm_map_ = *osm_map;
            ROS_INFO("OSM Map gotten");
            //ROS_INFO_STREAM("Got by topic: " << topological_map_);
        }
        */

    }

    void Osm2TopologicalMap::transformMap(){
        ROS_INFO("TODO");
    }

}
