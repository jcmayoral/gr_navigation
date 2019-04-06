#include <gr_map_utils/topological_to_metric_converter.h>

namespace gr_map_utils{
    Topological2MetricMap::Topological2MetricMap(ros::NodeHandle nh): nh_(nh), tf2_listener_(tf_buffer_),
                                                                    mark_nodes_(false), nodes_value_(127),
                                                                    inverted_costmap_(true), map_yaw_(0.0),
                                                                    map_offset_(2.0), cells_neighbors_(3){
        ROS_INFO("Initiliazing Node OSM2TopologicalMap Node");
        gr_tf_publisher_ = new TfFramePublisher();
        message_store_ = new mongodb_store::MessageStoreProxy(nh,"topological_maps");
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
        map_srv_client_ = nh_.serviceClient<geographic_msgs::GetGeographicMap>("get_geographic_map");\
        timer_publisher_ = nh_.createTimer(ros::Duration(0.1), &Topological2MetricMap::timer_cb, this);
        dyn_server_cb_ = boost::bind(&Topological2MetricMap::dyn_reconfigureCB, this, _1, _2);
      	dyn_server_.setCallback(dyn_server_cb_);
    }

    Topological2MetricMap::~Topological2MetricMap(){

    }

    void Topological2MetricMap::dyn_reconfigureCB(TopologicalMapConverterConfig &config, uint32_t level){
        mark_nodes_ = config.mark_nodes;
        nodes_value_ = config.nodes_value;
        inverted_costmap_ = config.invert_costmap;
        map_yaw_ = config.map_orientation;
        map_offset_ = config.map_offset;
        cells_neighbors_ = config.node_inflation;
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


        std::string name("simulation_map");

        if(message_store_->queryNamed<strands_navigation_msgs::TopologicalMap>(name,"map", results_map)) {
            BOOST_FOREACH( boost::shared_ptr<  strands_navigation_msgs::TopologicalMap> topological_map_,  results_map){
                ROS_INFO_STREAM("Got by name: " << *topological_map_);
            }
            return true;
        }



        std::vector< boost::shared_ptr<strands_navigation_msgs::TopologicalNode> > results;
        //On this version the map is stored by NAME Not anymore nodes stored
        ROS_WARN("QUERY NODES");
        if(message_store_->queryNamed<strands_navigation_msgs::TopologicalNode>(name,"map", results_node, false)) {
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
        map =  ros::topic::waitForMessage<strands_navigation_msgs::TopologicalMap>("static_topological_map", ros::Duration(3));
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
        created_map_.info.resolution = 0.1;

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
                //std::cout << "Edge ID "<< edges_it->edge_id << std::endl;
               
                char empty[it->name.size()];
                
                for (int c = 0; c< it->name.size(); c++)
                    empty[c] = '\0';
                
                std::string tmp_string = edges_it->edge_id.replace(edges_it->edge_id.begin(), edges_it->edge_id.begin() + it->name.size(), empty);
                std::cout << "aaa " << tmp_string << std::endl;
                
                std::string goal = edges_it->edge_id.substr(edges_it->edge_id.find("_") + 1);
                edges.emplace_back(it->name,goal);//This is just an example
                std::cout << it->name << " : " << goal << std::endl;
                //std::cout << it->name << "should be" << edges_it->edge_id.substr(0,edges_it->edge_id.find("_"));
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
            created_map_.data.resize(created_map_.info.width * created_map_.info.height,0);
        else
            created_map_.data.resize(created_map_.info.width * created_map_.info.height,255);

        float res = created_map_.info.resolution;

        //Update costs
        float range_x = max_x - min_x;
        float range_y = max_y - min_y;

        int index;
        int col;
        int row;

        if(mark_nodes_){
            for ( const std::pair<int,int>  &it : node_centers ){
                row = (it.first - origin.position.x)/res; //new_coordinate frame ...TODO Orientation
                col = (it.second - origin.position.y)/res;

                for (auto i = row-cells_neighbors_; i< row+cells_neighbors_; ++i){
                    for (auto j = col-cells_neighbors_; j< col+cells_neighbors_; ++j){
                        index = int(i + created_map_.info.width *j);
                        if (index > created_map_.data.size())
                            continue;
                        created_map_.data[index] = nodes_value_;
                    }
                }
            }

            //EDGES
            /*
            for (auto const& x : nodes_to_indexes){
              std::cout << x.first  // string (key)
              << ':' 
              << x.second.first // 
              << std::endl;
            }
            */

            //for (auto const& i : edges){ map
            float init_x, init_y;
            float dest_x, dest_y;
            float r; 
            double theta;
            int i_cell_x, i_cell_y, d_cell_x, d_cell_y;

            for (Edges & e  : edges){//= edges.begin(); i != edges.end(); i++){
                /*
                std::cout << "Edges from "
                <<  e.first 
                << " to "
                << e.second
                <<std::endl;
               */
                init_x = std::min<float>(fabs(nodes_coordinates[e.first].first), fabs(nodes_coordinates[e.second].first));
                init_y = std::min<float>(fabs(nodes_coordinates[e.first].second), fabs(nodes_coordinates[e.second].second));

                dest_x = std::max<float>(fabs(nodes_coordinates[e.first].first), fabs(nodes_coordinates[e.second].first));
                dest_y = std::max<float>(fabs(nodes_coordinates[e.first].second), fabs(nodes_coordinates[e.second].second));
                



                /*
                i_cell_x = (init_x - origin.position.x)/res;
                i_cell_y = (init_y- origin.position.y)/res;

                d_cell_x = (dest_x - origin.position.x)/res;
                d_cell_y = (dest_y - origin.position.y)/res;

                std::cout << "cell range x " << i_cell_x << ", " << d_cell_x << std::endl;
                std::cout << "cell range y " << i_cell_y << ", " << d_cell_y << std::endl;

                for (int i=i_cell_x; i<=d_cell_x-2; ++i)
                    for (int j=i_cell_y; j<=d_cell_y-2; ++j){
                        if (i<0 || j <0)
                            continue;
                        index = int(i + created_map_.info.width *j);
                        if (index > created_map_.data.size()){
                            ROS_ERROR("AAA");
                            continue;
                        }
                        created_map_.data[index] = 254;

                    }
                */

                theta = atan2(dest_y - init_y, dest_x - init_x);
                int dg = theta * 180 / M_PI;

                r = std::sqrt(std::pow(dest_y - init_y,2) + std::pow(dest_x - init_x,2));

                float r_x, r_y;
                for (float r_i = res; r_i <=r; r_i+=res){
                    r_x = r_i*cos(theta);
                    r_y = r_i*sin(theta);
                    row = round((init_x + r_x - origin.position.x)/res); 
                    col = round((init_y + r_y - origin.position.y)/res);
                   if (row<=0 || col <=0){
                       //ROS_ERROR("Negative index");
                       continue;
                   }

                    index = int(row + created_map_.info.width *col);
                    if (index > created_map_.data.size()){
                        //ROS_ERROR("INDEX TOO LARGE");
                        continue;
                    }

                    //std::cout << col << " and  " << row << std::endl;
                    created_map_.data[index] = 254;
                }

                /*

                for ( float i = init_x; i<=dest_x; i +=res ){
                    for ( float j = init_y; j<=dest_y; j+=res ){
                        std::cout << " i " << i << " j " << j << std::endl;
                        row = (i- origin.position.x)/res; 
                        col = (j- origin.position.y)/res;
                        index = int(row + created_map_.info.width *col);
                        if (index > created_map_.data.size())
                            continue;
                        created_map_.data[index] = 254;
                    }
                }
                */
            }
            std::cout << "ENDDDDDD";
            for (std::map<std::string, CellCoordinates >::iterator  it = nodes_coordinates.begin(); it!= nodes_coordinates.end(); ++it ){
                //std::printf("Edge from %s to %s /n", it.first, it.second);
                //std::cout <<"Edge from " << it->first << " to " << it->second << std::endl;
                
                //std::cout << "a";
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
        //gr_tf_publisher_->publishTfTransform();
    }

    void Topological2MetricMap::timer_cb(const ros::TimerEvent& event){
        publishMaps();
    }
}
