#include <gr_map_utils/osm_to_topological_converter.h>

namespace gr_map_utils{

    Osm2TopologicalMap::Osm2TopologicalMap(ros::NodeHandle nh): nh_(nh), osm_map_(), distance_to_origin_(100){
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

    bool Osm2TopologicalMap::getMapFromDatabase(){
        return false;
    }

    bool Osm2TopologicalMap::getMapFromTopic(){
        std::unique_lock<std::mutex> lk(mutex_);
        boost::shared_ptr<visualization_msgs::MarkerArray const> osm_map;
        osm_map =  ros::topic::waitForMessage<visualization_msgs::MarkerArray>("visualization_marker_array", ros::Duration(3.0));
        if (osm_map != NULL){
            osm_map_ = *osm_map;
            ROS_INFO("OSM Map gotten");
            return true;
            //ROS_INFO_STREAM("Got by topic: " << topological_map_);
        }
        return false;
    }

    bool Osm2TopologicalMap::getMapFromService(){
        return false;
    }

    void Osm2TopologicalMap::transformMap(){
        std::unique_lock<std::mutex> lk(mutex_);
        int count = 0;
        strands_navigation_msgs::TopologicalNode node;
        filtered_map_.markers.clear();
        topological_map_.nodes.clear();

        for (std::vector<visualization_msgs::Marker>::iterator it = osm_map_.markers.begin(); it != osm_map_.markers.end(); ++it){
            visualization_msgs::Marker marker(*it);
            marker.header.frame_id = "map";

            //This is a hack that MUST BE CORRECTED osm give UTM coordinates "world" however ROS do not recognize any world frame...
            //A quick fix might be publish a tf frame at origin. so this can be change to a tf2::doTransform fuction

            if (marker.pose.position.x > 0){
                marker.pose.position.x = marker.pose.position.x - gr_tf_publisher_->getOriginX();
                marker.pose.position.y = marker.pose.position.y - gr_tf_publisher_->getOriginY();
            }

            for (std::vector<geometry_msgs::Point>::iterator it_point = marker.points.begin() ; it_point != marker.points.end(); ++it_point){
                it_point->x = it_point->x - gr_tf_publisher_->getOriginX();
                it_point->y = it_point->y - gr_tf_publisher_->getOriginY();
            }

            filtered_map_.markers.emplace_back(marker);
            

            //if (it->type == visualization_msgs::Marker::CYLINDER){
            if (true){
                count ++;
                node.pose = it->pose;

                if(gr_tf_publisher_->getEuclideanDistanceToOrigin(node.pose.position.x, node.pose.position.y) < distance_to_origin_){
                    node.pose.position.x = node.pose.position.x - gr_tf_publisher_->getOriginX();
                    node.pose.position.y = node.pose.position.y - gr_tf_publisher_->getOriginY();
                    topological_map_.nodes.emplace_back(node);

                    for (std::vector<geometry_msgs::Point>::iterator it_point = it->points.begin() ; it_point != it->points.end(); ++it_point){
                        node.pose.position.x = it_point->x;
                        std::cout << "NODe " << node.pose.position.x << std::endl;
                        node.pose.position.y = it_point->y;
                        topological_map_.nodes.emplace_back(node);
                    }
                }

            }
        }
        //std::cout<< count ;
    }

    void Osm2TopologicalMap::osm_map_cb(const visualization_msgs::MarkerArray::ConstPtr& map){
        osm_map_ = *map;
    }

    void Osm2TopologicalMap::publishMaps(){
        //std::cout << "NODES " << topological_map_.nodes.size() << std::endl;
        topological_marker_pub_.publish(filtered_map_);
        topological_map_pub_.publish(topological_map_);
    }

}
