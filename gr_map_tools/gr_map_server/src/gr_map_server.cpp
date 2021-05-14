#include <gr_map_server/gr_map_server.h>

using namespace gr_map_server;

GRMapServer::GRMapServer(): nh_(""), map_frame_("map"){
    /*
    server_ = new actionlib::SimpleActionServer<GREdgesAction>(nh, "/topological/edges", boost::bind(&GRMapServer::execute_cb, _1, this), false);
    server_->start();
    */
    edges_sub_ = nh_.subscribe("region",10, &GRMapServer::edges_cb, this);
    map_metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("topological_map2", 1, true);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("topological_map2", 1, true);

    ros::spin();
}

void GRMapServer::execute_cb(const GREdgesActionGoal& goal){

}

void GRMapServer::edges_cb(const visualization_msgs::Marker::ConstPtr& region){
    if (region->points.size()!=4){
        ROS_ERROR("SKIP RECEIVED REGION");
        return;
    }

    for (auto point : region->points){
        std::cout << point.x << " ::: " << point.y << std::endl;
    }

}
