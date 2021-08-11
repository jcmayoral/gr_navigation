#include<gr_map_utils/osm_to_metric_converter.h>
#include<gr_map_utils/topological_to_metric_converter.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace gr_map_utils{
  class MapManager{
    
    protected:
      bool osm_required_;
    private:
      ros::Subscriber map_request_sub_;
      ros::NodeHandle nh_;
      gr_map_utils::Topological2MetricMap map_converter2_;
      gr_map_utils::Osm2MetricMap map_converter_;

    public:
      MapManager(ros::NodeHandle nh): nh_(nh), map_converter2_(nh_, false), map_converter_(nh_), osm_required_(false){
        map_request_sub_ = nh_.subscribe("osm_required",1, &MapManager::type_cb, this);
      }
      ~MapManager(){
      };

      bool prepareMaps(){
        if (!map_converter_.getMap()){
          ROS_ERROR("Map not gotten");
          return 1;
        }
        if (!map_converter2_.getMap()){
          ROS_ERROR("Map2 not gotten");
          return 1;
        }
        map_converter_.transformMap();
        map_converter2_.transformMap();
      }

      void type_cb(const std_msgs::Bool::ConstPtr type){
        osm_required_ = type->data;
      }

      void check_and_publish(){
        if(osm_required_){
          map_converter_.publishMaps();

        }
        else{
          map_converter2_.publishMaps();
        }
      }
    
  };
}
  