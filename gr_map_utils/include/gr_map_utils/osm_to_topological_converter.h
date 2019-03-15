#include <gr_map_utils/map_converter_interface.h>
#include <boost/foreach.hpp>
#include <mutex>
#include <vector>

#include <strands_navigation_msgs/TopologicalMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace gr_map_utils{
    
    class Osm2TopologicalMap : public MapConverterInterface{
        public:
            Osm2TopologicalMap(ros::NodeHandle nh);
            ~Osm2TopologicalMap();
            virtual bool storeMap();
            virtual bool getMapFromTopic();
            virtual bool getMapFromService();
            virtual bool getMapFromDatabase();
            virtual void transformMap();
            virtual void publishMaps();

            void osm_map_cb(const visualization_msgs::MarkerArray::ConstPtr& map);
        private:
            visualization_msgs::MarkerArray osm_map_;
            visualization_msgs::MarkerArray filtered_map_;
            strands_navigation_msgs::TopologicalMap static_topological_map_;
            strands_navigation_msgs::TopologicalMap topological_map_;
            ros::Publisher topological_map_pub_;
            ros::Publisher static_topological_map_pub_;
            ros::Publisher topological_marker_pub_;
            ros::Subscriber osm_map_sub_;
            ros::NodeHandle nh_;
            std::mutex mutex_;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf2_listener_;
            float distance_to_origin_;
    };
}
