#include <gr_map_utils/osm_to_topological_converter.h>

namespace gr_map_utils{

    Osm2TopologicalMap::Osm2TopologicalMap(ros::NodeHandle nh, std::string topic): nh_(nh), osm_map_(), distance_to_origin_(100),tf2_listener_(tf_buffer_){
        gr_tf_publisher_ = new TfFramePublisher();
        message_store_ = new mongodb_store::MessageStoreProxy(nh,"topological_maps");
        is_map_received_ = false;
        static_topological_map_pub_ = nh_.advertise<navigation_msgs::TopologicalMap>("static_topological_map", 1, true);
        topological_map_pub_ = nh_.advertise<navigation_msgs::TopologicalMap>("topological_map", 1, true);
        topological_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("filtered_topological_map", 1, true);
        osm_map_sub_ = nh_.subscribe(topic,10, &Osm2TopologicalMap::osm_map_cb, this);
        dyn_server_cb_ = boost::bind(&Osm2TopologicalMap::dyn_reconfigureCB, this, _1, _2);
      	dyn_server_.setCallback(dyn_server_cb_);
    }

    Osm2TopologicalMap::~Osm2TopologicalMap(){

    }

    void Osm2TopologicalMap::dyn_reconfigureCB(OSMMapConverterConfig &config, uint32_t level){
        ROS_INFO("aaaa");
        if (!is_map_received_){
            ROS_WARN("Map not received");
            return;
        }
        ROS_INFO("updating Map");
        //Update values
        distance_to_origin_ = config.distance_to_origin;
        transformMap();
    }


    bool Osm2TopologicalMap::storeMap(){
        std::string name("testing_topological_map");
        std::string id(message_store_->insertNamed(name, topological_map_));
        message_store_->updateID(id, topological_map_);
        return true;
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
            ROS_INFO("OSM Map gotten from topic");
            return true;
            //ROS_INFO_STREAM("Got by topic: " << topological_map_);
        }
        return false;
    }

    bool Osm2TopologicalMap::getMapFromService(){
        return false;
    }

    void Osm2TopologicalMap::transformMap(){
        //Ensure World frame exists
        gr_tf_publisher_->publishTfTransform();
        static_topological_map_.nodes.clear();
        topological_map_.nodes.clear();
        filtered_map_.markers.clear();


        std::cout << static_topological_map_.nodes.size()<< std::endl;
        //std::unique_lock<std::mutex> lk(mutex_);
        int count = 0;
        navigation_msgs::TopologicalNode node;
        filtered_map_.markers.clear();
        topological_map_.nodes.clear();
        geometry_msgs::TransformStamped to_map_transform; // My frames are named "base_link" and "leap_motion"
        geometry_msgs::PoseStamped out;
        geometry_msgs::PoseStamped in;

        std::string needle = "buildings_osm";
        std::string hack = "others_osm";

        for (std::vector<visualization_msgs::Marker>::iterator it = osm_map_.markers.begin(); it != osm_map_.markers.end(); ++it){
            visualization_msgs::Marker marker(*it);

            hack =  &marker.ns[0u];
            //transform world to map
            //topological topic does not contain frame_id
            in.header.frame_id = "world";
            in.pose.position.x = it->pose.position.x;
            in.pose.position.y = it->pose.position.y;
            in.pose.orientation.w = 1.0;
            node.pose = in.pose; //strands_nav "topological map"

            //osm server has some issues with frames some points come on world frame so quick fix(distance to origin > 10000) is implemented but must be changed
            if (gr_tf_publisher_->getEuclideanDistanceToOrigin(it->pose.position.x , it->pose.position.y) > 10000){//osm server has some issues with frames
                to_map_transform = tf_buffer_.lookupTransform("map", "world", ros::Time(0), ros::Duration(1.0) );
                tf2::doTransform(in, out, to_map_transform);
                marker.header = out.header;
                marker.pose = out.pose; //visualization_msgs "OSM Map"
                node.pose  = out.pose;
            }

            bool static_map = false;
            geometry_msgs::Point first_point, second_point;
            bool init = false;

            //find relevant point which are close to the origin
            if(gr_tf_publisher_->getEuclideanDistanceToOrigin(marker.pose.position.x , marker.pose.position.y) < distance_to_origin_){
                marker.header.frame_id = "map";
                filtered_map_.markers.emplace_back(marker);

                if (std::strcmp(needle.c_str(), hack.c_str()) == 0){// if building then pass to static map
                    static_topological_map_.nodes.emplace_back(node);
                    static_map = true;
                    //std::cout << it->points.size() << std::endl;
                }
                else{
                    topological_map_.nodes.emplace_back(node);
                }


                for (std::vector<geometry_msgs::Point>::iterator it_point = it->points.begin() ; it_point != it->points.end(); ++it_point){
                    //osm server has some issues with frames some points come on world frame so quick fix(distance to origin > 10000) is implemented but must be changed
                     if (gr_tf_publisher_->getEuclideanDistanceToOrigin(it_point->x , it_point->y) > 10000){//world coordinate detected
                        in.pose.position.x = it_point->x;
                        in.pose.position.y = it_point->y;
                        to_map_transform = tf_buffer_.lookupTransform("map", "world", ros::Time(0), ros::Duration(1.0) );
                        tf2::doTransform(in, out, to_map_transform);
                        node.pose.position.x = out.pose.position.x;
                        node.pose.position.y = out.pose.position.y;
                     }

                    if (static_map){//add point to static
                        static_topological_map_.nodes.emplace_back(node);
                    }
                    else //add point to global
                        topological_map_.nodes.emplace_back(node);

                    if(!init){//first point
                        first_point.x = node.pose.position.x;
                        first_point.y = node.pose.position.y;
                        init = true;
                    }
                    else{//add more point between vertices
                        second_point.x = node.pose.position.x;
                        second_point.y = node.pose.position.y;
                        float resolution_x = (second_point.x - first_point.x)/5;
                        float resolution_y = (second_point.y - first_point.y)/5;
                        float distance = sqrt(pow(second_point.x - first_point.x,2) + pow(second_point.y - first_point.y,2));
                        for(int i = 0; i < 3; i++){
                            node.pose.position.x = first_point.x + i*resolution_x;
                            node.pose.position.y = first_point.y + i*resolution_y;
                            if (static_map)
                                static_topological_map_.nodes.emplace_back(node);
                        }
                        //update vertexes
                        first_point.x = second_point.x;
                        first_point.y = second_point.y;
                    }
                }
            }
        }
                std::cout << static_topological_map_.nodes.size()<< std::endl;
    }

    void Osm2TopologicalMap::osm_map_cb(const visualization_msgs::MarkerArray::ConstPtr& map){
        osm_map_ = *map;
    }

    void Osm2TopologicalMap::publishMaps(){
        //std::cout << "NODES " << topological_map_.nodes.size() << std::endl;
        topological_marker_pub_.publish(filtered_map_);
        topological_map_pub_.publish(topological_map_);
        static_topological_map_pub_.publish(static_topological_map_);
        gr_tf_publisher_->publishTfTransform();
    }

}
