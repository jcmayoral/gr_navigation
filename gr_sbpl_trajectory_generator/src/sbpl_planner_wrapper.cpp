#include <gr_sbpl_trajectory_generator/spbl_planner_wrapper.h>

using namespace gr_sbpl_trajectory_generator;       

GRSBPLPlanner::GRSBPLPlanner(): nh_("~"), primitive_filename_(""){
    env_ = new EnvironmentNAVXYTHETALAT();
    ROS_INFO("Wait for Map");
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
    map_msg = ros::topic::waitForMessage< nav_msgs::OccupancyGrid>("map");
    ROS_INFO("Map Received");
    double nominalvel_mpersecs,timetoturn45degsinplace_secs;
    int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
    nh_.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    ros::param::get("~primitives_file", primitive_filename_);
    //TODO check why the next line does not work
    //nh_.param("primitives_file", primitive_filename_);
    nh_.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);
    ROS_INFO_STREAM("Primitives file on "<< primitive_filename_.c_str());

    lethal_obstacle_ = (unsigned char)  nh_.param("lethal_obstacle",20);;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;

    std::vector<geometry_msgs::Point> footprint = costmap_2d::makeFootprintFromParams(nh_);
    //makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint);
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);

    if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
      ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
      exit(1);
    }

    unsigned char cost_possibly_circumscribed_tresh = 0;
    if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_possibly_circumscribed_tresh))){
      ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }

    std::vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint.size());
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      perimeterptsV.push_back(pt);
    }
    double width=100;
    double height=100;
    double resolution=0.025;
    bool ret;
    try{
      ret = env_->InitializeEnv(width,
                                height,
                                0, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, resolution, nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, obst_cost_thresh,
                                primitive_filename_.c_str());
    }
    catch(SBPL_Exception e){
      ROS_ERROR_STREAM(e.what());
      ROS_ERROR("SBPL encountered a fatal exception!");
      ret = false;
    }

    ROS_ERROR("DONE FOR NOW");

}

unsigned char GRSBPLPlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}


GRSBPLPlanner::~GRSBPLPlanner(){

}