#include <gr_map_utils/topological_to_metric_converter.h>

namespace gr_map_utils{
    Topological2MetricMap::Topological2MetricMap(ros::NodeHandle nh): nh_(nh){
            ROS_INFO("Initiliazing Node OSM2TopologicalMap Node");
            message_store_ = new mongodb_store::MessageStoreProxy(nh);
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
            ROS_INFO_STREAM("Got by topic: " << topological_map_);
        }
    }
}
