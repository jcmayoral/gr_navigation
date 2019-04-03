#include <gr_safety_monitors/safe_actions/dynamic_reconfigure_safe_action.h>

using namespace gr_safety_monitors;

DynamicReconfigureSafeAction::DynamicReconfigureSafeAction(){
    ROS_INFO("Dynamic Reconfigure Safe Action");
};

DynamicReconfigureSafeAction::~DynamicReconfigureSafeAction(){
};

void DynamicReconfigureSafeAction::execute(){
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "max_trans_vel";
    double_param.value = 0.25;
    conf.doubles.push_back(double_param);

    double_param.name = "min_trans_vel";
    double_param.value = 0;
    conf.doubles.push_back(double_param);

    double_param.name = "max_vel_x";
    double_param.value = 0.2;
    conf.doubles.push_back(double_param);

    double_param.name = "accel_lim_x";
    double_param.value = 0.2;
    conf.doubles.push_back(double_param);

    double_param.name = "sim_time";
    double_param.value = 5.0;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
};

void DynamicReconfigureSafeAction::stop(){
    ROS_INFO("Stopping setting to default params");
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config conf;

    bool_param.name = "restore_defaults";
    bool_param.value = true;
    conf.bools.push_back(bool_param);

    srv_req.config = conf;

    ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
};
