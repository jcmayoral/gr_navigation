 #include <pluginlib/class_list_macros.h>
 #include "gr_cutting_global_planner/global_planner.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(gr_cutting_global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace gr_cutting_global_planner {
  GlobalPlanner::GlobalPlanner (){

  }
  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
     initialize(name, costmap_ros);
    }
  
  bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();
    float convert_offset = 0.0;

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset;
    my = (wy - origin_y) / resolution - convert_offset;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}


  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    costmap_ = boost::make_shared<costmap_2d::Costmap2D>(*costmap_ros->getCostmap());
  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
    
    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }

    plan.push_back(start);
    ROS_INFO_STREAM("MAP COORDS " << start_x_i  << " " << start_y_i);

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }

    ROS_INFO_STREAM("MAP COORDS " << goal_x_i  << " " << goal_y_i);

    /*
   for (int i=0; i<20; i++){
     geometry_msgs::PoseStamped new_goal = goal;
     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

      new_goal.pose.position.x = -2.5+(0.05*i);
      new_goal.pose.position.y = -3.5+(0.05*i);

      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w();

   plan.push_back(new_goal);
   }
   */
   plan.push_back(goal);
  return true;
 }
};

