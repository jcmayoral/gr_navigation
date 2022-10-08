#include <gr_line_trajectory_planner/line_trajectory_planner.h>

using namespace gr_line_trajectory_planner;

GRLinePlanner::GRLinePlanner(): nh_("~"), primitive_filename_(""), initial_epsilon_(1.0),
                                is_start_received_(false),action_name_("sbpl_action"),
                                odom_received_(false), tfBuffer(ros::Duration(5)),position_tolerance_(1.0),
                                tf2_listener(tfBuffer){
    //set goal_ yaw to zero
    goal_.pose.orientation.w = 1.0;

    as_ = new actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>(ros::NodeHandle(), action_name_, boost::bind(&GRLinePlanner::executeCB, this, _1), false);

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
    time_pub_ = nh_.advertise<std_msgs::Float32>("time_to_go", 1);

    ros::NodeHandle nh;
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("nav_vel", 1);

    point_sub_ = nh_.subscribe("clicked_point", 1, &GRLinePlanner::point_cb, this);
    odom_sub_ = nh.subscribe("/odometry/base_raw", 1, &GRLinePlanner::odom_cb, this);

    //ros::spinOnce();
    as_->start();
    ros::spin();

}


void GRLinePlanner::point_cb(const geometry_msgs::PointStampedConstPtr msg){
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
      ROS_INFO("Executing path");
      //executePath();
    }
    else{
      ROS_ERROR("ERROR");
    }

}


void GRLinePlanner::executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal){
  ROS_INFO_STREAM("Action " << action_name_ << " CALLED" );
  goal_ = goal->target_pose;

  setStart();

  if (makePlan(start_,goal_)){
    ROS_INFO("WORKING");
    if (executePath()){
      as_->setSucceeded();
    }
    else{
      as_->setAborted();
    }
  }
  else{
    ROS_ERROR("ERROR");
    as_->setAborted();
  }
}


unsigned char GRLinePlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}


GRLinePlanner::~GRLinePlanner(){
  delete as_;
}


void GRLinePlanner::setStart(){
  geometry_msgs::TransformStamped transformStamped;

  try{
    transformStamped = tfBuffer.lookupTransform("map", "base_link",
                             ros::Time::now());
    start_.header = transformStamped.header;
    start_.pose.position.x = transformStamped.transform.translation.x;
    start_.pose.position.y = transformStamped.transform.translation.y;
    start_.pose.orientation = transformStamped.transform.rotation;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;

  }
}

void GRLinePlanner::stop(){
  geometry_msgs::Twist cmd_vel;
  cmd_vel_pub_.publish(cmd_vel);
}

double GRLinePlanner::getRotationInFrame(geometry_msgs::PoseStamped& pose, std::string frame){
  geometry_msgs::TransformStamped transform_stamped;
  //tf::Pose tf_pose;
  transform_stamped = tfBuffer.lookupTransform(frame, pose.header.frame_id, ros::Time::now(), ros::Duration(1.0) );
  tf2::doTransform(pose, pose, transform_stamped);
  //tf::poseMsgToTF(pose.pose, tf_pose);
  return tf2::getYaw(pose.pose.orientation);
}



bool GRLinePlanner::executePath(){
  geometry_msgs::TransformStamped base_link_to_map;

  double yaw1, yaw2;
  geometry_msgs::PoseStamped current_pose;

  while(plan_.size()>1){
    ros::Duration(0.1).sleep();
    if (odom_received_){
      odom_received_ = false;
    }
    else{
      continue;
    }

    geometry_msgs::Twist cmd_vel;
    //Transforming next waypoint on map_coordinates to base_libk
    plan_[0].header.stamp = ros::Time::now();
    base_link_to_map = tfBuffer.lookupTransform("base_link", "map", ros::Time::now(), ros::Duration(1.0) );
    tf2::doTransform(plan_[0], plan_[0], base_link_to_map);

    //Calculating Angles
    // Difference between base_link and plan_[0] on base_link
    yaw2 = getRotationInFrame(plan_[0], "base_link");

    //Difference between expected and actual position on time t (P Controller)
    cmd_vel.linear.x = plan_[0].pose.position.x;

    if (cmd_vel.linear.x > position_tolerance_ || cmd_vel.linear.y > position_tolerance_){
      stop();
      return false;
    }

    cmd_vel.linear.y = plan_[0].pose.position.y;
    cmd_vel.angular.z = yaw2;

    //D Controller TODO configurable
    if (plan_.size()>2){
      cmd_vel.linear.x -= 0.2*(plan_[1].pose.position.x - plan_[0].pose.position.x);
    }

    //TODO I Controller
    //this store the last pose while finish
    current_pose = plan_[0];
    plan_.erase(plan_.begin());

    cmd_vel_pub_.publish(cmd_vel);
    ROS_INFO_STREAM_THROTTLE(2,"Time to GO " << plan_.size()*0.1);
    ROS_ERROR_STREAM(cmd_vel);
    std_msgs::Float32 fb_msg;
    fb_msg.data = plan_.size()*0.1;
    time_pub_.publish(fb_msg);
  }

  stop();


  //orientation of current odometry to map
  geometry_msgs::TransformStamped base_link_to_odom;
  geometry_msgs::PoseStamped p;
  p.header = odom_msg_.header;
  p.header.stamp = ros::Time::now();
  p.pose = odom_msg_.pose.pose;
  base_link_to_odom = tfBuffer.lookupTransform("base_link", "map", ros::Time::now(), ros::Duration(1.0) );
  tf2::doTransform(p, p, base_link_to_odom);

  yaw1 = getRotationInFrame(p, "map");

  //orientation of the goal on map frame
  yaw2 = getRotationInFrame(goal_, "map");

  geometry_msgs::Twist vel;
  while (abs(yaw2-yaw1) > 0.15){//TODO this should be reconfigurable
    //TODO min
    vel.angular.z = (yaw2 - yaw1)/2;
    cmd_vel_pub_.publish(vel);
    ROS_ERROR_STREAM("Correcting "<< yaw2 - yaw1);
    ros::Duration(0.05).sleep();

    //orientation of current odometry to map
    geometry_msgs::TransformStamped base_link_to_odom;
    geometry_msgs::PoseStamped p;
    p.header = odom_msg_.header;
    p.header.stamp = ros::Time::now();
    p.pose = odom_msg_.pose.pose;
    base_link_to_odom = tfBuffer.lookupTransform("base_link",  odom_msg_.header.frame_id, ros::Time::now(), ros::Duration(1.0) );
    tf2::doTransform(p, p, base_link_to_odom);

    yaw1 = getRotationInFrame(p, "map");
  }

  stop();
  ROS_INFO("Motion Completed");
  return true;
}

void GRLinePlanner::odom_cb(const nav_msgs::OdometryConstPtr odom_msg){
  odom_received_ = true;
  odom_msg_ = *odom_msg;
}


bool GRLinePlanner::makePlan(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal){

  plan_.clear();
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  double theta_mid = (theta_goal-theta_start)/2;

  auto angle = atan2(goal.pose.position.y - start.pose.position.y, goal.pose.position.x - start.pose.position.x);
  auto acceleration = 0.5;
  auto dt = 0.1;

  geometry_msgs::Pose mid_pose;
  mid_pose.position.x = start.pose.position.x + (goal.pose.position.x - start.pose.position.x)/2;
  mid_pose.position.y = start.pose.position.y + (goal.pose.position.y - start.pose.position.y)/2;

  nav_msgs::Path gui_path;

  geometry_msgs::PoseStamped current_pose;
  current_pose = start;
  current_pose.pose.orientation = goal.pose.orientation;
  int c = 0;
  double velocity = 0.0;
  double maxvel = 2.0;
  std::vector<geometry_msgs::PoseStamped> oneway_midpath;

  auto startgoal_dist_half = dist2goal(start.pose, goal.pose)/2.0;
  std::cout << "startgoal hald distance " << startgoal_dist_half <<std::endl;

  if (dist2goal(start.pose, goal.pose)< 0.2){
    ROS_INFO_STREAM("ROBOT ON GOAL POse");
    return true;
  }

  
  while (dist2goal(current_pose.pose, goal_.pose)>= startgoal_dist_half){
    std::cout << dist2goal(current_pose.pose, mid_pose) << std::endl;
    oneway_midpath.push_back(current_pose);
    //std::cout << oneway_midpath.size() << std::endl;
    current_pose.pose.position.x += cos(angle)*acceleration*dt;
    current_pose.pose.position.y += sin(angle)*velocity;
    current_pose.pose.orientation = goal.pose.orientation;
    velocity += acceleration*dt;
    if (velocity > maxvel){
      velocity = maxvel;
    }
    c++;
  }
  ROS_ERROR_STREAM("OUT path size" << oneway_midpath.size());


  /*
  auto angle_diff = theta_mid/oneway_midpath.size();
  auto count = 1;
  tf2::Quaternion current_quaternion;

  for ( auto pose : oneway_midpath){
    current_quaternion.setRPY(0,0,theta_start);
    current_quaternion.normalize();
    tf2::convert( pose.pose.orientation, current_quaternion);
    //startyaw += angle_diff;
    //pose.pose.orientation.w = 1.0;
  }
  */

  
  //gui_path.poses.insert(gui_path.end(), gui_path.poses.rbegin(), gui_path.poses.rend());


  /*
  std::vector<geometry_msgs::PoseStamped> reverse_midpath;
  reverse_midpath.reserve(oneway_midpath.size());
  gui_path.poses.reserve(2*oneway_midpath.size());

  //THIS IS NOT WORKING CHECK IT LATER
  std::copy(oneway_midpath.begin(), oneway_midpath.end(), gui_path.poses.begin());//std::back_inserter(gui_path.poses));
  std::reverse_copy(oneway_midpath.begin(), oneway_midpath.end(), reverse_midpath.begin());//std::back_inserter(gui_path.poses));
  std::copy(reverse_midpath.begin(), reverse_midpath.end(), gui_path.poses.begin()+oneway_midpath.size());//std::back_inserter(gui_path.poses));
  */

  //HaCK
  geometry_msgs::PoseStamped tmp;

  for (int i=0; i< oneway_midpath.size(); i++){
    //gui_path.poses[i].header.stamp = ros::Time::now();
    //gui_path.poses[i].header.frame_id = "map";//costmap_ros_->getGlobalFrameID();
    //gui_path.poses[i] = oneway_midpath[i];
    tmp = oneway_midpath[i];
    tmp.header.stamp = ros::Time::now();
    tmp.header.frame_id = "map";
    //tmp.pose.orientation.w = 1.0;
    gui_path.poses.push_back(tmp);
  }

  auto xoffset = oneway_midpath[oneway_midpath.size()-1].pose.position.x;
  auto yoffset = oneway_midpath[oneway_midpath.size()-1].pose.position.y;
  

  for (int i=oneway_midpath.size()-1; i> 0; i--){
    //gui_path.poses[i].header.stamp = ros::Time::now();
    //gui_path.poses[i].header.frame_id = "map";//costmap_ros_->getGlobalFrameID();
    //gui_path.poses[i] = oneway_midpath[i]
    tmp = oneway_midpath[i];
    tmp.header.stamp = ros::Time::now();
    tmp.header.frame_id = "map";
    tmp.pose.position.x +=xoffset;
    tmp.pose.position.y +=yoffset;
    //tmp.pose.orientation.w = 1.0;
    //TODO ANGLE
    gui_path.poses.push_back(tmp);
  }
  ROS_ERROR_STREAM("GUT path size" << gui_path);

  //gui_path.poses.resize(sbpl_path.size());
  gui_path.header.frame_id = "map";//costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = ros::Time::now();
  for(auto pose: gui_path.poses){
    geometry_msgs::PoseStamped ppose;
    ppose.header.stamp = ros::Time::now();
    ppose.header.frame_id = "map";//costmap_ros_->getGlobalFrameID();

    ppose.pose.position.x = pose.pose.position.x;
    ppose.pose.position.y = pose.pose.position.y;
    ppose.pose.position.z = pose.pose.position.z;

    //tf2::Quaternion temp;
    //temp.setRPY(0,0,0);
    //pose.pose.orientation.x = temp.getX();
    //pose.pose.orientation.y = temp.getY();
    //pose.pose.orientation.z = temp.getZ();
    //pose.pose.orientation.w = temp.getW();
    ppose.pose.orientation = pose.pose.orientation;

    plan_.push_back(ppose);

    //gui_path.poses[i] = plan_[i];
  }
  plan_pub_.publish(gui_path);
  ROS_INFO_STREAM(gui_path);



  
  /*
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
    //ROS_ERROR(e->what());
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
  */
  //create a message for the plan
 
  //publishStats(solution_cost, sbpl_path.size(), start, goal);
  return true;
};