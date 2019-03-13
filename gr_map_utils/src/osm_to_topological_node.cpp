#include <gr_map_utils/osm_to_topological_converter.h>
using namespace gr_map_utils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "osm_to_topological_converter");
    ros::NodeHandle nh;
    Osm2TopologicalMap map_converter(nh);
    return 0;
}
