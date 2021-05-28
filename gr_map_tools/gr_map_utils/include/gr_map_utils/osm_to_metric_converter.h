#include <gr_map_utils/map_converter_interface.h>
#include <gr_map_utils/OSMMapConverterConfig.h>

#include <boost/foreach.hpp>
#include <mutex>
#include <vector>

#include <navigation_msgs/TopologicalMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/server.h>

//TO TEST
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>

//YAML
#include <yaml-cpp/yaml.h>
#include <nav_msgs/GetMap.h>

namespace gr_map_utils{

    static grid_map::GridMap OSMGRIDMAP{};

    class Osm2MetricMap : public MapConverterInterface{
        public:
            Osm2MetricMap(ros::NodeHandle nh,std::string config_file="config/default_osm.yaml");
            ~Osm2MetricMap();
            virtual bool storeMap();
            virtual bool getMapFromTopic();
            virtual bool getMapFromService();
            virtual bool getMapFromDatabase();
            virtual void transformMap();
            virtual void publishMaps();
            void publishTransform();

            bool addOSMRegions();
            void osm_map_cb(const visualization_msgs::MarkerArray::ConstPtr& map);
            void dyn_reconfigureCB(OSMMapConverterConfig &config, uint32_t level);
            void fillPolygon(std::vector<double>x, std::vector<double> y);
            bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res );
        private:
            visualization_msgs::MarkerArray osm_map_;
            ros::Publisher topological_map_pub_;

            //TO BE TESTED
            ros::Publisher gridmap_pub_;

            ros::Subscriber osm_map_sub_;
            ros::NodeHandle nh_;
            std::mutex mutex_;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf2_listener_;
            float distance_to_origin_;
            dynamic_reconfigure::Server<OSMMapConverterConfig> dyn_server_;
            dynamic_reconfigure::Server<OSMMapConverterConfig>::CallbackType dyn_server_cb_;
            mongodb_store::MessageStoreProxy* message_store_;

            //Reuse gridmap to meteric
            grid_map::GridMap gridmap_;
            nav_msgs::OccupancyGrid grid_;

            //Integrate Map Service
            ros::ServiceServer get_map_service_;

            bool is_ready_;
            std::string in_topic_;
            std::string needle_;
            std::string type_;
            std::string map_frame_;

    };
}
