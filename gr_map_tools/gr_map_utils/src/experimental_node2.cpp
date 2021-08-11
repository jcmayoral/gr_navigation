#include <gr_map_utils/map_manager.hpp>
#include <ros/ros.h>

using namespace gr_map_utils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "osm_to_metric_experimental");
    ros::NodeHandle nh;
    MapManager map_manager(nh);
   
    ros::Rate loop_rate(10);

    while (ros::ok()){
        loop_rate.sleep();
        map_manager.check_and_publish();
        ros::spinOnce();
    }

    return 0;
}
