#include <gr_map_utils/topological_to_metric_converter.h>

namespace gr_map_utils{
    Topological2MetricMap::Topological2MetricMap(ros::NodeHandle nh){
            ROS_INFO("Initiliazing Node OSM2TopologicalMap Node");
            message_store_ = new mongodb_store::MessageStoreProxy(nh);
    }

    Topological2MetricMap::~Topological2MetricMap(){

    }

    bool Topological2MetricMap::storeMap(){
        std::string name("mypose");
        geometry_msgs::Pose pose;
        std::string id(message_store_->insertNamed(name, pose));

        //message_store_.updateID(id, topological_map_);
        return true;
    }

    bool Topological2MetricMap::getMap(){
        //std::vector< boost::shared_ptr<strands_navigation_msgs::TopologicalNode> > results;
        //std::string id(message_store_.insertNamed("my_map", topological_map_));

        /*
        if(message_store_.queryNamed<strands_navigation_msgs::TopologicalNode>("my_map", results)) {
            ROS_INFO("A");
            /*BOOST_FOREACH( boost::shared_ptr<  strands_navigation_msgs::TopologicalNode> topological_map_,  results){
                ROS_INFO_STREAM("Got by name: " << *topological_map_);
            }
        }

        results.clear();

        if(message_store_.queryID<strands_navigation_msgs::TopologicalNode>(id, results)) {
            BOOST_FOREACH( boost::shared_ptr<strands_navigation_msgs::TopologicalNode> topological_map_,  results){
                ROS_INFO_STREAM("Got by ID: " << *topological_map_);
            }
            return true;
        }
        */

        return false;
    }
}
