#include <ros/ros.h>
#include <gr_action_msgs/GREdgesAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace gr_action_msgs;

namespace gr_map_server{
    class GRMapServer{
        public:
            GRMapServer();
            ~GRMapServer(){
            };
        protected:
            void execute_cb(const GREdgesActionGoal& goal);
        private:
            ros::NodeHandle nh_;
            boost::shared_ptr<actionlib::SimpleActionServer<GREdgesAction>> server_;
            ros::Topic
    };
}