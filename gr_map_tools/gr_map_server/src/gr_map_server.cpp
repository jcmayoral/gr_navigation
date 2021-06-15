#include <gr_map_server/gr_map_server.h>

using namespace gr_map_server;

GRMapServer::GRMapServer(): nh_(""), map_frame_("map"){
    /*
    server_ = new actionlib::SimpleActionServer<GREdgesAction>(nh, "/topological/edges", boost::bind(&GRMapServer::execute_cb, _1, this), false);
    server_->start();
    */
    edges_sub_ = nh_.subscribe("region",10, &GRMapServer::edges_cb, this);
    map_metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    //message_store_ = new mongodb_store::MessageStoreProxy(nh_,"frames","message_store");
    ros::spin();
}

void GRMapServer::execute_cb(const GREdgesActionGoal& goal){

}

void GRMapServer::edges_cb(const visualization_msgs::Marker::ConstPtr& region){
    if (region->points.size()<4){
        ROS_ERROR_STREAM("SKIP RECEIVED REGION"<< region->points.size());
        return;
    }

    auto minx = std::numeric_limits<float>::max();
    auto maxx = std::numeric_limits<float>::min();

    auto miny = std::numeric_limits<float>::max();
    auto maxy = std::numeric_limits<float>::min();

    for (auto point : region->points){
        std::cout << point.x << " ::: " << point.y << std::endl;
        if (point.x < minx){
            minx = point.x;
        }
        if (point.x > maxx){
            maxx = point.x;
        }
        if (point.y < miny){
            miny = point.y;
        }
        if (point.y > maxy){
            maxy = point.y;
        }
    }

    auto width = maxx-minx;
    auto height = maxy-miny;
    auto resolution = 0.025;

    //std::cout << "HEIGHT " << height << " c " << ceil(height/) <<std::endl;
    //std::cout << "WIDHT " << width << " c "<< ceil(width/0.025)<< std::endl;
    metadata_.map_load_time = ros::Time::now();
    metadata_.width = ceil(width/resolution);
    metadata_.height = ceil(height/resolution);
    metadata_.resolution = resolution;

    metadata_.origin.position.x = minx;
    metadata_.origin.position.y = miny;
    
    metadata_.origin.orientation.w =1.0;

    map_.header.frame_id = map_frame_;
    map_.header.stamp = ros::Time::now();
    map_.info = metadata_;
    

    auto cells = ceil(height/resolution)*ceil(width/resolution);    
    map_.data.clear();
    for (int i = 0; i< cells;i++)
        map_.data.push_back(0);
      
    //map_.data = map;

    map_pub_.publish(map_);
    map_metadata_pub_.publish(metadata_);

}
