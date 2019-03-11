#include <gr_map_utils/topological_to_metric_converter.h>

namespace gr_map_utils{
    Topological2MetricMap::Topological2MetricMap(ros::NodeHandle nh): nh_(nh){
            ROS_INFO("Initiliazing Node OSM2TopologicalMap Node");
            message_store_ = new mongodb_store::MessageStoreProxy(nh);
            map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
            metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    }

    Topological2MetricMap::~Topological2MetricMap(){

    }

    bool Topological2MetricMap::storeMap(){
        std::string name("my_map");
        std::string id(message_store_->insertNamed(name, topological_map_));
        message_store_->updateID(id, topological_map_);
        return true;
    }

    bool Topological2MetricMap::getMap(){
        std::vector< boost::shared_ptr<strands_navigation_msgs::TopologicalNode> > results;

        std::string id(message_store_->insertNamed("simulation_map", topological_map_));

        if(message_store_->queryNamed<strands_navigation_msgs::TopologicalNode>("simulation_map", results)) {
            BOOST_FOREACH( boost::shared_ptr<  strands_navigation_msgs::TopologicalNode> topological_map_,  results){
                ROS_INFO_STREAM("Got by name: " << *topological_map_);
                return true;
            }
        }

        results.clear();

        if(message_store_->queryID<strands_navigation_msgs::TopologicalNode>(id, results)) {
            BOOST_FOREACH( boost::shared_ptr<strands_navigation_msgs::TopologicalNode> topological_map_,  results){
                ROS_INFO_STREAM("Got by ID: " << *topological_map_);
            }
            return true;
        }

        ROS_ERROR("Map not gotten");
        return false;
    }

    void Topological2MetricMap::getMapFromTopic(){
        ROS_INFO("Wait map from topic");
        boost::shared_ptr<strands_navigation_msgs::TopologicalMap const> map;
        map =  ros::topic::waitForMessage<strands_navigation_msgs::TopologicalMap>("topological_map");
        if (map != NULL){
            topological_map_ = *map;
            //ROS_INFO_STREAM("Got by topic: " << topological_map_);
        }
    }

    void Topological2MetricMap::convertTopologicalMap(){
        nav_msgs::OccupancyGrid created_map;
        created_map.header.stamp = ros::Time::now();
        created_map.header.frame_id = "map"; //TODO this should be a param

        created_map.info.map_load_time = ros::Time::now();
        created_map.info.resolution = 0.05;
        created_map.info.width = 400;//TODO
        created_map.info.height = 400;

        //TODO
        created_map.data.resize(created_map.info.width * created_map.info.height);

        geometry_msgs::Pose origin;//TODO
        origin.orientation.w = 1.0;

        created_map.info.origin = origin;

        nav_msgs::MapMetaData meta_data_message;
        meta_data_message = created_map.info;
        map_pub_.publish(created_map);
        metadata_pub_.publish(meta_data_message);
    }
}
