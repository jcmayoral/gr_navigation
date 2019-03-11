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
        std::unique_lock<std::mutex> lk(mutex_);

        created_map_.header.frame_id = "map"; //TODO this should be a param
        created_map_.info.resolution = 0.05;

        int nodes_number = 0;
        float center_x = 0.0;
        float center_y = 0.0;
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();

        float max_x = 0.0;
        float max_y = 0.0;

        float node_x;
        float node_y;

        for (std::vector<strands_navigation_msgs::TopologicalNode>::iterator it = topological_map_.nodes.begin(); it!= topological_map_.nodes.end(); ++it){
            node_x = it->pose.position.x;
            node_y = it->pose.position.y;

            center_x += node_x;

            if (node_x < min_x){
                min_x = node_x;
            }

            if(node_x > max_x){
                max_x = node_x;
            }

            center_y += node_y;

            if (node_y < min_y){
                min_y = node_y;
            }

            if(node_y > max_y){
                max_y = node_y;
            }

            nodes_number ++;
        }

        created_map_.info.width = int( (max_x - min_x)/created_map_.info.resolution );
        created_map_.info.height =  int( (max_y - min_y)/created_map_.info.resolution );
        
        //TODO
        created_map_.data.resize(created_map_.info.width * created_map_.info.height);

        geometry_msgs::Pose origin;
        origin.position.x = min_x;
        origin.position.y = min_y;
        origin.orientation.w = 1.0;

        created_map_.info.origin = origin;

        ROS_INFO("Map Created");
    }

    void Topological2MetricMap::publishMaps(){
        std::unique_lock<std::mutex> lk(mutex_);
        created_map_.header.stamp = ros::Time::now();
        created_map_.info.map_load_time = ros::Time::now();

        nav_msgs::MapMetaData meta_data_message;
        meta_data_message = created_map_.info;
        map_pub_.publish(created_map_);
        metadata_pub_.publish(meta_data_message);
    }
}
