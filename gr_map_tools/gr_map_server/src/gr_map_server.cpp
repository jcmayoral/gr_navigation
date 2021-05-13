#include <gr_map_server/gr_map_server.h>

using namespace gr_map_server;

GRMapServer::GRMapServer(): nh_(""){
    server_ = new actionlib::SimpleActionServer<GREdgesAction>(nh, "/topological/edges", boost::bind(&GRMapServer::execute_cb, _1, this), false);
    server_->start();
    ros::spin();
}

void GRMapServer::execute_cb(const GREdgesActionGoal& goal){

}