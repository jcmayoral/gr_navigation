#include <gr_map_utils/topological_to_metric_converter.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topological_to_metric_converter");
    ros::NodeHandle nh;
    gr_map_utils::Topological2MetricMap map_converter(nh);
    map_converter.storeMap();
    return 0;
}
