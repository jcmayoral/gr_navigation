#include <ros/ros.h>
#include <gr_action_msgs/GREdgesAction.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <visualization_msgs/Marker.h>

using namespace gr_action_msgs;

namespace gr_map_server{
    class GRMapServer{
        public:
            GRMapServer();
            ~GRMapServer(){
            };
        protected:
            void execute_cb(const GREdgesActionGoal& goal);
            void edges_cb(const visualization_msgs::Marker::ConstPtr& region);
        private:
            ros::NodeHandle nh_;
            boost::shared_ptr<actionlib::SimpleActionServer<GREdgesAction>> server_;
            ros::Subscriber edges_sub_;
            ros::Publisher map_pub_;
            ros::Publisher map_metadata_pub_;
            std::string map_frame_;
            bool init_;
            nav_msgs::OccupancyGrid map_;
            nav_msgs::MapMetaData metadata_;
    };
}