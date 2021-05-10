#include <gr_map_utils/topological_to_metric_converter.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topological_to_metric_converter");
    ros::NodeHandle nh;
    // Not publish the tf by default
    gr_map_utils::Topological2MetricMap map_converter(nh, false);

    if (false) //TODO
        map_converter.storeMap();

    if (!map_converter.getMap()){
        ROS_ERROR("Map not gotten");
        return 1;
    }

    map_converter.transformMap();
    ros::ServiceServer update_map_service_ = nh.advertiseService("update_metric_map", &gr_map_utils::Topological2MetricMap::updateMap, &map_converter);
    ros::spin();
    //while (ros::ok()){
    //    ROS_INFO_ONCE("WORK");
    //}

    return 0;
}
