#include <gr_map_server/gr_map_server.h>
#include <ros/ros.h>
using namespace gr_map_server;

int main(int argc, char**argv){
    ros::init(argc, argv,"gr_map_server");
    GRMapServer server;
}