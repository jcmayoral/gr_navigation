#ifndef GR_TOPO_CONVERTER_GRIDMAP
#define GR_TOPO_CONVERTER_GRIDMAP

#include <gr_map_utils/map_converter_interface.h>
#include <gr_map_utils/UpdateMap.h>
#include <gr_map_utils/TopologicalMapConverterConfig.h>
#include <boost/foreach.hpp>
#include <mutex>
#include <vector>
#include <math.h>
#include <dynamic_reconfigure/server.h>

#include <navigation_msgs/TopologicalMap.h>
#include <geometry_msgs/Pose.h>
#include <geographic_msgs/GetGeographicMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

namespace gr_map_utils{

    typedef std::pair<float,float> CellCoordinates;
    typedef std::pair<std::string,std::string> Edges;

    #ifndef GR_OSM_GRIDMAP
    #define GR_OSM_GRIDMAP
    static grid_map::GridMap OSMGRIDMAP{};
    #endif

    class Topological2MetricMap : public MapConverterInterface{
        public:
            Topological2MetricMap(ros::NodeHandle nh, bool initialize_tf =true, bool init_dyn=true);
            ~Topological2MetricMap();
            virtual bool storeMap();
            virtual bool getMapFromTopic();
            virtual bool getMapFromService();
            virtual bool getMapFromDatabase();
            virtual void transformMap();
            virtual void publishMaps();
            bool updateMap(UpdateMap::Request &req, UpdateMap::Response &resp);
            void timer_cb(const ros::TimerEvent& event);
            void dyn_reconfigureCB(TopologicalMapConverterConfig &config, uint32_t level);
            void parseMessage(navigation_msgs::TopologicalMap& map, TopoMapMetaPtr msg);

        private:
            navigation_msgs::TopologicalMap topological_map_;
            nav_msgs::OccupancyGrid created_map_;
            ros::Subscriber topological_map_sub_;
            ros::Publisher map_pub_;
            ros::Publisher metadata_pub_;
            ros::NodeHandle nh_;
            std::mutex mutex_;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf2_listener_;
            ros::ServiceClient map_srv_client_;
            ros::Timer timer_publisher_;
            dynamic_reconfigure::Server<TopologicalMapConverterConfig> dyn_server_;
            dynamic_reconfigure::Server<TopologicalMapConverterConfig>::CallbackType dyn_server_cb_;
            bool mark_nodes_;
            bool mark_edges_;
            bool inverted_costmap_;
            int nodes_value_;
            int edges_value_;
            int invert_value_;
            double map_yaw_;
            double map_resolution_;
            float map_offset_;
            int cells_neighbors_;

            //Reuse gridmap to meteric
            grid_map::GridMap gridmap_;
            nav_msgs::OccupancyGrid grid_;
            ros::Publisher gridmap_pub_;
            bool is_ready_;

    };
}
#endif
