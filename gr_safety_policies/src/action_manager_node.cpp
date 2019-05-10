#include <pluginlib/class_loader.h>
#include <safety_core/safe_action.h>
#include <memory>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_manager_node");

  int action_selected = -1;
  std::map<int, std::string> action_map_;
  boost::shared_ptr<safety_core::SafeAction> action;
  std::string desired_str;
  pluginlib::ClassLoader<safety_core::SafeAction> action_loader("safety_core", "safety_core::SafeAction");

  try {
    std::cout << "Select Action to activate (-1 to abort) "<< std::endl;

    std::vector<std::string> classes = action_loader.getDeclaredClasses();
    for(unsigned int i = 0; i < classes.size(); ++i){
      std::cout << i << ":" << classes[i] << std::endl;
      action_map_[i] = classes[i];
    }

    std::cin >> action_selected;

    if (action_selected == -1){
      ROS_WARN("ABORTING");
      exit(0);
    }

    desired_str = classes[action_selected];

    if(action_loader.isClassAvailable(desired_str)){
      ROS_INFO("Available Action... Loading");
      action = action_loader.createInstance(desired_str.c_str());
      ROS_INFO("Created safe_action %s", classes[action_selected].c_str());
      action->execute();
      action->stop();
      action.reset();
    }
  }
  catch (const pluginlib::PluginlibException& ex){
    ROS_FATAL("Failed to create action %s", ex.what());
    exit(1);
  }

}
