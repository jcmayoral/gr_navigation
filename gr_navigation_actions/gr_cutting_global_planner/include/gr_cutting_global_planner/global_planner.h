/** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>

 #include <tf/tf.h>

 using std::string;

 #ifndef GR_CUTTING_GLOBAL_PLANNER_CPP
 #define GR_CUTTING_GLOBAL_PLANNER_CPP

namespace gr_cutting_global_planner {
    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
        public:
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);
        bool worldToMap(double wx, double wy, double& mx, double& my);

        private: 
        boost::shared_ptr<costmap_2d::Costmap2D> costmap_;
    };
};
 #endif

