#include <gr_sbpl_trajectory_generator/spbl_planner_wrapper.h>

using namespace gr_sbpl_trajectory_generator;

GRSBPLPlanner::GRSBPLPlanner(): nh_("~"), primitive_filename_(""), initial_epsilon_(1.0),is_start_received_(false){
    env_ = new EnvironmentNAVXYTHETALAT();
    planner_ = new ARAPlanner(env_, true); //forward_search
    ROS_INFO("Wait for Map");
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
    map_msg = ros::topic::waitForMessage< nav_msgs::OccupancyGrid>("map");
    map_metadata_ = ros::topic::waitForMessage< nav_msgs::MapMetaData>("map_metadata");

    ROS_INFO("Map Received");
    double nominalvel_mpersecs,timetoturn45degsinplace_secs;
    int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
    nh_.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    ros::param::get("~primitives_file", primitive_filename_);
    nh_.param("initial_epsilon",initial_epsilon_,3.0);
    nh_.param("allocated_time", allocated_time_, 3.0);

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
    double resolution=map_metadata_->resolution;
    bool ret;
    try{
      ret = env_->InitializeEnv(map_metadata_->width,
                                map_metadata_->height,
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


    plan_pub_ = nh_.advertise<nav_msgs::Path>("plan", 1);
    point_sub_ = nh_.subscribe("clicked_point", 1, &GRSBPLPlanner::point_cb, this);

    //ros::spinOnce();
    ros::spin();

}


void GRSBPLPlanner::point_cb(const geometry_msgs::PointStampedConstPtr msg){
  if (is_start_received_){
    ROS_INFO("Receiving Goal");
    goal_.header = msg->header;
    goal_.pose.position = msg->point;
    goal_.pose.orientation.w = 1.0;
  }
  else{
    ROS_INFO("Receiving Start");
    start_.header = msg->header;
    start_.pose.position = msg->point;
    start_.pose.orientation.w = 1.0;
    is_start_received_ = true;
    return;
  }


  if (makePlan(start_,goal_)){
      ROS_INFO("WORKING");
    }
    else{
      ROS_ERROR("ERROR");
    }

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





bool GRSBPLPlanner::makePlan(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal){
  std::vector<geometry_msgs::PoseStamped> plan;
  plan.clear();

  ROS_INFO("[sbpl_lattice_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  try{
    //Check conversion offset of map start with frame -> gr_map_utils
    //Substract offset

    ROS_INFO_STREAM("start "<< start.pose.position.x - map_metadata_->origin.position.x << " , " <<  start.pose.position.y - map_metadata_->origin.position.y );
    int ret = env_->SetStart(start.pose.position.x - map_metadata_->origin.position.x, start.pose.position.y - map_metadata_->origin.position.y, theta_start);

    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception *e){
    ROS_ERROR(e->what());
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  try{
    //Check conversion offset of map start with frame -> gr_map_utils
    //Substract offset

   ROS_INFO_STREAM("goal "<< goal.pose.position.x - map_metadata_->origin.position.x << " , " <<  goal.pose.position.y - map_metadata_->origin.position.y );

    int ret = env_->SetGoal(goal.pose.position.x - map_metadata_->origin.position.x, goal.pose.position.y - map_metadata_->origin.position.y , theta_goal);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception *e){
    ROS_ERROR(e->what());
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }


  int offOnCount = 0;
  int onOffCount = 0;
  int allCount = 0;
  vector<nav2dcell_t> changedcellsV;

  for(unsigned int ix = 0; ix < map_metadata_->width; ix++) {
    for(unsigned int iy = 0; iy < map_metadata_->height; iy++) {

      unsigned char oldCost = env_->GetMapCost(ix,iy);
      unsigned char newCost = costMapCostToSBPLCost(0);

      if(oldCost == newCost) continue;

      allCount++;

      //first case - off cell goes on

      if((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        offOnCount++;
      }

      if((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        onOffCount++;
      }
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(0));//costmap_ros_->getCostmap()->getCost(ix,iy)));

      nav2dcell_t nav2dcell;
      nav2dcell.x = ix;
      nav2dcell.y = iy;
      changedcellsV.push_back(nav2dcell);
    }
  }

  try{
    if(!changedcellsV.empty()){
      ROS_ERROR("This matters");
      //StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
      //planner_->costs_changed(*scq);
      //delete scq;
    }

    //if(allCount > force_scratch_limit_)
      //planner_->force_planning_from_scratch();
  }
  catch(SBPL_Exception *e){
    ROS_ERROR("SBPL failed to update the costmap");
    return false;
  }

  //setting planner parameters
  //ROS_DEBUG("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(true);

  ROS_DEBUG("[sbpl_lattice_planner] run planner");
  vector<int> solution_stateIDs;
  int solution_cost;
  try{
    int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
    if(ret)
      ROS_DEBUG("Solution is found\n");
    else{
      ROS_INFO("Solution not found\n");
      //publishStats(solution_cost, 0, start, goal);
      return false;
    }
  }
  catch(SBPL_Exception *e){
    ROS_ERROR("SBPL encountered a fatal exception while planning");
    return false;
  }

  ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());

  vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  try{
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  }
  catch(SBPL_Exception *e){
    ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  // if the plan has zero points, add a single point to make move_base happy
  if( sbpl_path.size() == 0 ) {
    //Substract offset
    EnvNAVXYTHETALAT3Dpt_t s(
        start.pose.position.x,
        start.pose.position.y,
        theta_start);
    sbpl_path.push_back(s);
  }

  ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());
  ros::Time plan_time = ros::Time::now();

  //create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(sbpl_path.size());
  gui_path.header.frame_id = "map";//costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = plan_time;
  for(unsigned int i=0; i<sbpl_path.size(); i++){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = "map";//costmap_ros_->getGlobalFrameID();

    pose.pose.position.x = sbpl_path[i].x + map_metadata_->origin.position.x;
    pose.pose.position.y = sbpl_path[i].y + map_metadata_->origin.position.y;
    pose.pose.position.z = start.pose.position.z;

    tf2::Quaternion temp;
    temp.setRPY(0,0,sbpl_path[i].theta);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    plan.push_back(pose);

    gui_path.poses[i] = plan[i];
  }
  plan_pub_.publish(gui_path);
  //ROS_INFO_STREAM(gui_path);
  //publishStats(solution_cost, sbpl_path.size(), start, goal);
  return true;
};
