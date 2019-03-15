#include <gr_map_utils/topological_to_metric_converter.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topological_to_metric_converter");
    ros::NodeHandle nh;
    gr_map_utils::Topological2MetricMap map_converter(nh);

    ros::Rate loop_rate(10);

    if (false) //TODO
        map_converter.storeMap();
    

    if (!map_converter.getMap()){
        ROS_ERROR("Map not gotten");
        return 1;
    }

    map_converter.transformMap();
    
    while (ros::ok()){
        loop_rate.sleep();
        map_converter.publishMaps();
    }

    return 0;
}
