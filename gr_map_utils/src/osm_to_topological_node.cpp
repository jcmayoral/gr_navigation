#include <gr_map_utils/osm_to_topological_converter.h>
using namespace gr_map_utils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "osm_to_topological_converter");
    Osm2TopologicalMap map_converter;
    return 0;
}
