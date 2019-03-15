#include <gr_map_utils/osm_to_topological_converter.h>

namespace gr_map_utils{

    Osm2TopologicalMap::Osm2TopologicalMap(ros::NodeHandle nh): nh_(nh), osm_map_(), distance_to_origin_(100),tf2_listener_(tf_buffer_){
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
        //Ensure World frame exists
        gr_tf_publisher_->publishTfTransform();

        std::unique_lock<std::mutex> lk(mutex_);
        int count = 0;
        strands_navigation_msgs::TopologicalNode node;
        filtered_map_.markers.clear();
        topological_map_.nodes.clear();
        geometry_msgs::TransformStamped to_map_transform; // My frames are named "base_link" and "leap_motion"
        geometry_msgs::PoseStamped out;
        geometry_msgs::PoseStamped in;


        for (std::vector<visualization_msgs::Marker>::iterator it = osm_map_.markers.begin(); it != osm_map_.markers.end(); ++it){
            visualization_msgs::Marker marker(*it);
            marker.header.frame_id = "map";
            

            //transform world to map
            in.header.frame_id = it->header.frame_id;//todo topological map should include frame_id
            in.pose.position.x = it->pose.position.x;
            in.pose.position.y = it->pose.position.y;
            in.pose.orientation.w = 1.0;
            to_map_transform = tf_buffer_.lookupTransform("map", it->header.frame_id, ros::Time(0), ros::Duration(1.0) );
            tf2::doTransform(in, out, to_map_transform);
            marker.pose = out.pose; //visualization_msgs "OSM Map"
            node.pose = out.pose; //strands_nav "topological map"


            //transform poses as well
            for (std::vector<geometry_msgs::Point>::iterator it_point = marker.points.begin() ; it_point != marker.points.end(); ++it_point){
                in.pose.position.x = it_point->x;
                in.pose.position.y = it_point->y;
                to_map_transform = tf_buffer_.lookupTransform("map", it->header.frame_id, ros::Time(0), ros::Duration(1.0) );
                tf2::doTransform(in, out, to_map_transform);
                it_point->x = out.pose.position.x;
                it_point->y = out.pose.position.y;

            }

            if(gr_tf_publisher_->getEuclideanDistanceToOrigin(marker.pose.position.x , marker.pose.position.y) < distance_to_origin_){
                count ++;
                if (true){
                //if (it->type == visualization_msgs::Marker::LINE_STRIP){
                    node.pose.position.x = node.pose.position.x;
                    node.pose.position.y = node.pose.position.y;
                    topological_map_.nodes.emplace_back(node);
                    filtered_map_.markers.emplace_back(marker);

                    for (std::vector<geometry_msgs::Point>::iterator it_point = it->points.begin() ; it_point != it->points.end(); ++it_point){
                        node.pose.position.x = it_point->x;
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
        gr_tf_publisher_->publishTfTransform();
    }

}
