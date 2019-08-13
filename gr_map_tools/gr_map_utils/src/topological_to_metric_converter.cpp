#include <gr_map_utils/topological_to_metric_converter.h>

namespace gr_map_utils{
    Topological2MetricMap::Topological2MetricMap(ros::NodeHandle nh): nh_(nh), tf2_listener_(tf_buffer_),
                                                                    mark_nodes_(false), mark_edges_(false),
                                                                    nodes_value_(127), edges_value_(127),
                                                                    inverted_costmap_(true), map_yaw_(0.0),
                                                                    map_offset_(2.0), cells_neighbors_(3),
                                                                    map_resolution_(0.1){
        ROS_INFO("Initiliazing Node Topological2MetricMap Node");
        gr_tf_publisher_ = new TfFramePublisher();
        message_store_ = new mongodb_store::MessageStoreProxy(nh,"topological_maps");
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
        //Not implemented yet
        map_srv_client_ = nh_.serviceClient<geographic_msgs::GetGeographicMap>("get_geographic_map");
        timer_publisher_ = nh_.createTimer(ros::Duration(0.1), &Topological2MetricMap::timer_cb, this);
        dyn_server_cb_ = boost::bind(&Topological2MetricMap::dyn_reconfigureCB, this, _1, _2);
      	dyn_server_.setCallback(dyn_server_cb_);
    }

    Topological2MetricMap::~Topological2MetricMap(){

    }

    void Topological2MetricMap::dyn_reconfigureCB(TopologicalMapConverterConfig &config, uint32_t level){
        mark_nodes_ = config.mark_nodes;
        mark_edges_ = config.mark_edges;
        nodes_value_ = config.nodes_value;
        edges_value_ = config.edges_value;
        inverted_costmap_ = config.invert_costmap;
        map_yaw_ = config.map_orientation;
        map_offset_ = config.map_offset;
        cells_neighbors_ = config.feature_inflation;
        map_resolution_ = config.map_resolution;
        transformMap();
    }

    bool Topological2MetricMap::updateMap(UpdateMap::Request &req, UpdateMap::Response &resp){
        //std::unique_lock<std::mutex> lk(mutex_);
        ROS_INFO("Inside Service CB");
        getMap();
        transformMap();
        resp.success = true;
        return true;
    }

    bool Topological2MetricMap::storeMap(){
        std::string name("testing_metric_map");
        /*
        topological_map_.name = name;
        topological_map_.map = name;
        std::string id(message_store_->insertNamed(name, topological_map_));
        ROS_INFO_STREAM("ID "<< id);
        //message_store_->updateID(id, topological_map_);
        */
        return true;
    }

    bool Topological2MetricMap::getMapFromDatabase(){
        ROS_INFO("Trying getting map from database");
        std::vector< boost::shared_ptr<strands_navigation_msgs::TopologicalMap> > results_map;
        std::vector< boost::shared_ptr<strands_navigation_msgs::TopologicalNode> > results_node;

        std::string name("wish_map_move_base");

        if(message_store_->queryNamed<strands_navigation_msgs::TopologicalMap>(name,"map", results_map)) {
            topological_map_.nodes.clear();
            BOOST_FOREACH( boost::shared_ptr<  strands_navigation_msgs::TopologicalMap> topological_map_,  results_map){
                ROS_INFO_STREAM("Got by name: " << *topological_map_);
            }
            return true;
        }



        std::vector< boost::shared_ptr<strands_navigation_msgs::TopologicalNode> > results;
        //On this version the map is stored by NAME Not anymore nodes stored
        ROS_WARN("QUERY NODES");
        if(message_store_->queryNamed<strands_navigation_msgs::TopologicalNode>(name,"map", results_node, false)) {
            topological_map_.nodes.clear();
            strands_navigation_msgs::TopologicalNode node;
            BOOST_FOREACH( boost::shared_ptr<  strands_navigation_msgs::TopologicalNode> node,  results_node){
                ROS_DEBUG_STREAM("Got by name: " << *node);
                topological_map_.nodes.push_back(*node);
            }
           return true;
        }

        //results.clear();
        /*
        if(message_store_->queryID<strands_navigation_msgs::TopologicalMap>("5c40a11a81c5d07de7bb289f", results_map)) {
            ROS_INFO("1");
            BOOST_FOREACH( boost::shared_ptr<strands_navigation_msgs::TopologicalMap> topological_map_,  results_map){
                ROS_INFO_STREAM("Got by ID: " << *topological_map_);
            }
            return true;
        }
        if(message_store.queryID<strands_navigation_msgs::TopologicalNode>("5c40a11a81c5d07de7bb289f", results)) {
            for (std::vector< boost::shared_ptr<strands_navigation_msgs::TopologicalNode> >::iterator it = results.begin();it!=results.end(); it++){
                std::cout << "Name: " << (boost::static_pointer_cast<strands_navigation_msgs::TopologicalNode>(*it))->name << std::endl;

            }
            BOOST_FOREACH( boost::shared_ptr<  strands_navigation_msgs::TopologicalNode> map,  results){
                ROS_INFO_STREAM("Got by name: " << *map);
            }
            //return true; for testing

        }
        */
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
        ROS_INFO("Trying getting Map from Service");
        return false;
    }


    void Topological2MetricMap::transformMap(){
        std::unique_lock<std::mutex> lk(mutex_);
        created_map_.data.clear();
        created_map_.header.frame_id = "map"; //TODO this should be a param
        created_map_.info.resolution = map_resolution_;

        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();

        float max_x = 0.0;
        float max_y = 0.0;

        float node_x;
        float node_y;

        std::vector<CellCoordinates> node_centers;
        std::map<std::string, CellCoordinates > nodes_coordinates;
        std::vector<Edges> edges;

        geometry_msgs::TransformStamped to_map_transform; // My frames are named "base_link" and "leap_motion"
        geometry_msgs::PoseStamped out;
        geometry_msgs::PoseStamped in;

        for (std::vector<strands_navigation_msgs::TopologicalNode>::iterator it = topological_map_.nodes.begin(); it!= topological_map_.nodes.end(); ++it){
            //std::cout << "node name "<< it->name << std::endl;
            in.header.frame_id = "map";//todo topological map should include frame_id
            in.pose.position.x = it->pose.position.x;
            in.pose.position.y = it->pose.position.y;
            in.pose.orientation.w = 1.0;
            to_map_transform = tf_buffer_.lookupTransform("map", in.header.frame_id, ros::Time(0), ros::Duration(1.0) );
            tf2::doTransform(in, out, to_map_transform);
            node_x = out.pose.position.x;
            node_y = out.pose.position.y;


            if (node_x < min_x){
                min_x = node_x;
            }

            if(node_x > max_x){
                max_x = node_x;
            }


            if (node_y < min_y){
                min_y = node_y;
            }

            if(node_y > max_y){
                max_y = node_y;
            }

            node_centers.emplace_back(node_x, node_y);
            nodes_coordinates[it->name] = CellCoordinates(node_x,node_y);

            for (std::vector<strands_navigation_msgs::Edge>::iterator edges_it = it->edges.begin(); edges_it!= it->edges.end(); ++edges_it){
                //Since some node names has several "_" the beginning of the array must be replaced or erased with null values
                //Assuming edges convention name as edge_id_1  " _ " edge_id_2

                //Create copy of edge name
                std::string tmp_string(edges_it->edge_id);
                //remove edge_id_1;
                tmp_string.erase(tmp_string.begin(), tmp_string.begin() + it->name.size());
                // get goal edge_id_2
                std::string goal = tmp_string.substr(tmp_string.find("_") + 1);
                //Push_back edges
                edges.emplace_back(it->name,goal);//This is just an example
            }
        }


        geometry_msgs::Pose origin;
        origin.position.x = min_x - map_offset_/2;
        origin.position.y = min_y - map_offset_/2;

        //No rule map must match orientation of map frame
        tf2::Quaternion tmp_quaternion;
        tmp_quaternion.setRPY( 0, 0, map_yaw_ );
        // or for the other conversion direction
        origin.orientation = tf2::toMsg(tmp_quaternion);

        created_map_.info.origin = origin;
        created_map_.info.width = int( (max_x - min_x)/created_map_.info.resolution ) + int(map_offset_/created_map_.info.resolution);
        created_map_.info.height =  int( (max_y - min_y)/created_map_.info.resolution ) + int(map_offset_/created_map_.info.resolution);

        if (inverted_costmap_)
            created_map_.data.resize(created_map_.info.width * created_map_.info.height,254);
        else
            created_map_.data.resize(created_map_.info.width * created_map_.info.height,0);

        float res = created_map_.info.resolution;

        //Update costs
        float range_x = max_x - min_x;
        float range_y = max_y - min_y;

        int index;
        int col;
        int row;

        if(mark_nodes_){

            //Mark nodes
            for ( const std::pair<int,int>  &it : node_centers ){
                row = (it.first - origin.position.x)/res; //new_coordinate frame ...TODO Orientation
                col = (it.second - origin.position.y)/res;

                //inflate nodes on map
                for (auto i = row-cells_neighbors_; i<= row+cells_neighbors_; i++){
                    for (auto j = col-cells_neighbors_; j<= col+cells_neighbors_; j++){
                        index = int(i + created_map_.info.width *j);
                        if (index > created_map_.data.size())
                            continue;
                        created_map_.data[index] = nodes_value_;
                    }
                }
            }
        }

        if(mark_edges_){
            //variables used for edges
            double init_x, init_y;
            double dest_x, dest_y;
            float r;
            double theta;
            float r_x, r_y;
            //Iterates edges
            for (Edges & e  : edges){

                //Establish Limits
                init_x = nodes_coordinates[e.first].first;
                init_y = nodes_coordinates[e.first].second;

                dest_x = nodes_coordinates[e.second].first;
                dest_y = nodes_coordinates[e.second].second;

                //Edge Angle
                theta = atan2(dest_y - init_y, dest_x - init_x);

                //Distance between nodes
                r = std::sqrt(std::pow(dest_y - init_y,2) + std::pow(dest_x - init_x,2));

                //inflate edges
                for (int inflation=-cells_neighbors_;inflation<= cells_neighbors_;inflation++){
                    for (double r_i = 0; r_i <=r; r_i+=res){
                        //Local Coordiantes
                        r_x = r_i*cos(theta);
                        r_y = r_i*sin(theta);
                        //To Index
                        row = round((init_x + r_x - origin.position.x)/res);
                        col = round((init_y + r_y - origin.position.y)/res);
                        index = int(row+inflation + created_map_.info.width *(col+inflation));
                        if (index > created_map_.data.size()){
                            continue;
                        }

                        created_map_.data[index] = edges_value_;

                        //HACK
                        //Avoid undesired marks on costmap
                        index = int(row + created_map_.info.width *(col+inflation));
                        if (index > created_map_.data.size()){
                            continue;
                        }
                        created_map_.data[index] = edges_value_;
                        index = int(inflation+row + created_map_.info.width *(col));
                        if (index > created_map_.data.size()){
                            continue;
                        }
                        created_map_.data[index] = edges_value_;
                    }
                }
            }
        }
        ROS_INFO("Map Created");
    }

    void Topological2MetricMap::publishMaps(){
        created_map_.header.stamp = ros::Time::now();
        created_map_.info.map_load_time = ros::Time::now();

        nav_msgs::MapMetaData meta_data_message;
        meta_data_message = created_map_.info;
        map_pub_.publish(created_map_);
        metadata_pub_.publish(meta_data_message);

        //TODO CHECK feasibility condition if already exists do nothing
        gr_tf_publisher_->publishTfTransform();
    }

    void Topological2MetricMap::timer_cb(const ros::TimerEvent& event){
        publishMaps();
    }
}
