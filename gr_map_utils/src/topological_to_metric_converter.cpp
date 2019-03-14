#include <gr_map_utils/topological_to_metric_converter.h>

namespace gr_map_utils{
    Topological2MetricMap::Topological2MetricMap(ros::NodeHandle nh): nh_(nh), tf2_listener_(tf_buffer_){
            ROS_INFO("Initiliazing Node OSM2TopologicalMap Node");
            gr_tf_publisher_ = new TfFramePublisher();
            message_store_ = new mongodb_store::MessageStoreProxy(nh);
            map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
            metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
            map_srv_client_ = nh_.serviceClient<geographic_msgs::GetGeographicMap>("get_geographic_map");
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

        std::string id(message_store_->insertNamed("utm_topological_map", topological_map_));

        if(message_store_->queryNamed<strands_navigation_msgs::TopologicalNode>("utm_topological_map", results)) {
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

        if (getMapFromTopic()){
            ROS_INFO("Retrieving map from topic");
            return true;
        }

        if(getMapFromService()){
            ROS_INFO("Retrieving map from service");
            return true;
        }

        return false;
    }

    bool Topological2MetricMap::getMapFromTopic(){
        ROS_INFO("Wait map from topic.. timeout to 3 seconds");
        boost::shared_ptr<strands_navigation_msgs::TopologicalMap const> map;
        map =  ros::topic::waitForMessage<strands_navigation_msgs::TopologicalMap>("topological_map", ros::Duration(3));
        if (map != NULL){
            topological_map_ = *map;
            //ROS_INFO_STREAM("Got by topic: " << topological_map_);
            return true;
        }
        return false;
    }

    bool Topological2MetricMap::getMapFromService(){
        return false;
    }

    void Topological2MetricMap::transformMap(){
        std::unique_lock<std::mutex> lk(mutex_);

        created_map_.header.frame_id = "map"; //TODO this should be a param
        created_map_.info.resolution = 0.1;
        float offset = 2; //TODO should be a parameter
        int neighbors = 3;// TODO

        int nodes_number = 0;
        float center_x = 0.0;
        float center_y = 0.0;
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();

        float max_x = 0.0;
        float max_y = 0.0;

        float node_x;
        float node_y;

        std::vector<std::pair<int,int> > cells;
        geometry_msgs::TransformStamped to_map_transform; // My frames are named "base_link" and "leap_motion"
        geometry_msgs::PoseStamped out;
        geometry_msgs::PoseStamped in;

        for (std::vector<strands_navigation_msgs::TopologicalNode>::iterator it = topological_map_.nodes.begin(); it!= topological_map_.nodes.end(); ++it){
            
            in.header.frame_id = "map";//todo topological map should include frame_id
            in.pose.position.x = it->pose.position.x;
            in.pose.position.y = it->pose.position.y;
            in.pose.orientation.w = 1.0;
            to_map_transform = tf_buffer_.lookupTransform("map", "map", ros::Time(0), ros::Duration(1.0) );
            tf2::doTransform(in, out, to_map_transform);
            
            node_x = out.pose.position.x;
            node_y = out.pose.position.y;

            std::cout << "x " << node_x;

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
            cells.emplace_back(node_x, node_y);
        }


        geometry_msgs::Pose origin;
        origin.position.x = min_x - offset/2;
        origin.position.y = min_y - offset/2;
        origin.orientation.w = 1.0;

        created_map_.info.origin = origin;
        created_map_.info.width = int( (max_x - min_x)/created_map_.info.resolution ) + int(offset/created_map_.info.resolution);
        created_map_.info.height =  int( (max_y - min_y)/created_map_.info.resolution ) + int(offset/created_map_.info.resolution);
        
        created_map_.data.resize(created_map_.info.width * created_map_.info.height);

        float res = created_map_.info.resolution;

        //Update costs
        float range_x = max_x - min_x;
        float range_y = max_y - min_y;

        int index;
        int col;
        int row;

        for ( const std::pair<int,int>  &it : cells ){
            row = (it.first - origin.position.x)/res; //new_coordinate frame ...TODO Orientation
            col = (it.second - origin.position.y)/res;
            
            for (auto i = row-neighbors; i< row+neighbors; ++i){
                for (auto j = col-neighbors; j< col+neighbors; ++j){
                    index = int(i + created_map_.info.width *j);
                    created_map_.data[index] = 127;
                }
            }
        }

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

        //TODO CHECK feasibility condition if already exists do nothing
        //gr_tf_publisher_->publishTfTransform();
    }
}
