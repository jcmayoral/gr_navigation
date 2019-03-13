#include <gr_map_utils/osm_to_topological_converter.h>

namespace gr_map_utils{

    Osm2TopologicalMap::Osm2TopologicalMap(ros::NodeHandle nh): nh_(nh), osm_map_(){
        gr_tf_publisher_ = new TfFramePublisher();
        message_store_ = new mongodb_store::MessageStoreProxy(nh);
        topological_map_pub_ = nh_.advertise<strands_navigation_msgs::TopologicalMap>("topological_map", 1, true);
        topological_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("topological_map_2", 1, true); 

        osm_map_sub_ = nh_.subscribe("visualization_marker_array",10, &Osm2TopologicalMap::osm_map_cb, this);
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
        std::unique_lock<std::mutex> lk(mutex_);
        boost::shared_ptr<visualization_msgs::MarkerArray const> osm_map;
        osm_map =  ros::topic::waitForMessage<visualization_msgs::MarkerArray>("visualization_marker_array");
        if (osm_map != NULL){
            osm_map_ = *osm_map;
            ROS_INFO("OSM Map gotten");
            //ROS_INFO_STREAM("Got by topic: " << topological_map_);
        }
    }

    void Osm2TopologicalMap::transformMap(){
        std::unique_lock<std::mutex> lk(mutex_);
        int count = 0;
        strands_navigation_msgs::TopologicalNode node;
        filtered_map_.markers.clear();
        topological_map_.nodes.clear();

        for (std::vector<visualization_msgs::Marker>::iterator it = osm_map_.markers.begin(); it != osm_map_.markers.end(); ++it){
             ROS_INFO("TODO");
            if (it->type == visualization_msgs::Marker::CYLINDER){
                count ++;
                node.pose = it->pose;
                filtered_map_.markers.emplace_back(*it);
                topological_map_.nodes.emplace_back(node);
            }
        }
        std::cout<< count ;
    }

    void Osm2TopologicalMap::osm_map_cb(const visualization_msgs::MarkerArray::ConstPtr& map){
        osm_map_ = *map;
    }

    void Osm2TopologicalMap::publishMaps(){
        std::cout << "NODES " << topological_map_.nodes.size() << std::endl;
        topological_marker_pub_.publish(filtered_map_);
        topological_map_pub_.publish(topological_map_);
    }

}
