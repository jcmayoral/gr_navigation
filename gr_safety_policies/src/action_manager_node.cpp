#include <pluginlib/class_loader.h>
#include <safety_core/safe_action.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_manager_node");

  try {
    pluginlib::ClassLoader<safety_core::SafeAction> action_loader("safety_core", "safety_core::SafeAction");
    //check if a non fully qualified name has potentially been passed in
    if(true){//}!action_loader.isClassAvailable(fault_detector_)){
      std::vector<std::string> classes = action_loader.getDeclaredClasses();
      for(unsigned int i = 0; i < classes.size(); ++i){
        std::cout << classes[i] << std::endl;


        /*
        if(desired_id == fd_loader_.getName(classes[i])){
        //if we've found a match... we'll get the fully qualified name and break out of the loop
        ROS_WARN("Fault Detector specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
            fault_detector_.c_str(), classes[i].c_str());
        fault_detector_ = classes[i];
        break;
        */
      }
    }
  //}

  /*
  fd_ = fd_loader_.createInstance(fault_detector_);
  ROS_INFO("Created fault_detector_ %s", fault_detector_.c_str());
  //fd_->initialize(fd_loader_.getName(fault_detector_));
  fd_->instantiateServices(nh);
  */
} catch (const pluginlib::PluginlibException& ex)
{
  ROS_FATAL("Failed to create the %s detector", ex.what());
  exit(1);
}

}
