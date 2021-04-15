#include <gr_map_utils/osm_to_metric_converter.h>
using namespace gr_map_utils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "osm_to_metric_experimental");
    ros::NodeHandle nh;
    Osm2MetricMap map_converter(nh);
    Osm2MetricMap map_converter_2(nh, "config/edges.yaml");

    ros::Rate loop_rate(10);


    if (!map_converter.getMap()){
        ROS_ERROR("Map not gotten");
        return 1;
    }
    if (!map_converter_2.getMap()){
        ROS_ERROR("Map2 not gotten");
        return 1;
    }

    map_converter.transformMap();
    auto layers = OSMGRIDMAP.getLayers();

    for (auto l:layers){
      std::cout << " LAYER "<< l << std::endl;
    }

    map_converter.addOSMRegions();
    map_converter_2.transformMap();
    //auto layer = OSMGRIDMAP.get("example");
    //map_converter_2.addLayer("example", layer);
    //map_converter.storeMap();

    while (ros::ok()){
        loop_rate.sleep();
        map_converter.publishMaps();
        //map_converter_2.publishMaps();
        ros::spinOnce();
    }

    return 0;
}
