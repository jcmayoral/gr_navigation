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

  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    costmap_ = boost::make_shared<costmap_2d::Costmap2D>(*costmap_ros->getCostmap());
  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
    
    double wxs = start.pose.position.x;
    double wys = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i, mid_x_i, mid_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wxs, wys, start_x_i, start_y_i)) {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }

    plan.push_back(start);
    ROS_INFO_STREAM("Start WORLD COORDS " << wxs << " " << wys);
    ROS_INFO_STREAM("Start MAP COORDS " << start_x_i  << " " << start_y_i);

    double wxg = goal.pose.position.x;
    double wyg = goal.pose.position.y;

    if (!costmap_->worldToMap(wxg, wyg, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }

    ROS_INFO_STREAM("Goal WORLD COORDS " << wxg << " " << wyg);
    ROS_INFO_STREAM("Goal MAP COORDS " << goal_x_i  << " " << goal_y_i);

    double midx = wxs + (wxg-wxs)/2;
    double midy = wys + (wyg-wys)/2;
    ROS_WARN_STREAM("mid "<< midx <<  " "  << midy);

    costmap_->worldToMap(midx,midy, mid_x_i, mid_y_i);
    ROS_INFO_STREAM("MID COORDS " << mid_x_i  << " " << mid_y_i);
    ROS_INFO_STREAM("Middle WORLD COORDS " << midx  << " " << midy);

    geometry_msgs::PoseStamped mid_pose;
    mid_pose.header = goal.header;
    mid_pose.pose.position.x = midx;
    mid_pose.pose.position.y = midy;
    mid_pose.pose.orientation = goal.pose.orientation;
    
    plan.push_back(goal);

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

