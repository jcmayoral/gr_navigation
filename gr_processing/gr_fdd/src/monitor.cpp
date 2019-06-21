#include <gr_fdd/monitor.h>

void MainMonitor::empty_cb(const std_msgs::EmptyConstPtr msg, int index){
    data_containers_[index].updateData(0,0);
    data_containers_[index].updateTime();
    
}

void MainMonitor::float_cb(const std_msgs::Float64ConstPtr msg, int index){
    data_containers_[index].updateData(msg->data);
    data_containers_[index].updateTime();
}

void MainMonitor::twist_cb(const geometry_msgs::TwistConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->linear.x,0);
    data_containers_[index].updateData(msg->angular.z,1);
}   

void MainMonitor::odom_cb(const nav_msgs::OdometryConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->twist.twist.linear.x,0);
    data_containers_[index].updateData(msg->twist.twist.angular.z,1);
}    

void MainMonitor::map_cb(const nav_msgs::OccupancyGridConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->info.origin.position.x,0);
    data_containers_[index].updateData(msg->info.origin.position.y,1);
}

void MainMonitor::imu_cb(const sensor_msgs::ImuConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->linear_acceleration.x,0);
    data_containers_[index].updateData(msg->angular_velocity.z,1);
}

void MainMonitor::joints_cb(const sensor_msgs::JointStateConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->position[0],0);
    data_containers_[index].updateData(msg->position[2],1);
}

void MainMonitor::fix_cb(const sensor_msgs::NavSatFixConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->latitude,0);
    data_containers_[index].updateData(msg->longitude,1);
    data_containers_[index].updateData(msg->altitude,1);
}

void MainMonitor::in_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int index, std::string topic_name){
    const std::string& datatype   = msg->getDataType();
    //const std::string& definition = msg->getMessageDefinition();
    if (datatype.compare("std_msgs/Empty") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const std_msgs::Empty::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::empty_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);            
    }
    
    if (datatype.compare("geometry_msgs/Twist") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const geometry_msgs::Twist::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::twist_cb, this, _1, index) ;      
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);            
    }

    if (datatype.compare("nav_msgs/Odometry") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const nav_msgs::Odometry::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::odom_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);            
    }

    if (datatype.compare("nav_msgs/OccupancyGrid") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::map_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);            
    }

    if (datatype.compare("sensor_msgs/Imu") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const sensor_msgs::Imu::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::imu_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);
    }
    
    if (datatype.compare("sensor_msgs/JointState") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const sensor_msgs::JointState::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::joints_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);
    }
    
    if (datatype.compare("std_msgs/Float64") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const std_msgs::Float64::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::float_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);
    }

    if (datatype.compare("sensor_msgs/NavSatFix") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const sensor_msgs::NavSatFix::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::fix_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);
    }
    ROS_INFO("Monitor started");
    //ROS_INFO_STREAM(msg->getMessageDefinition());
    //ROS_INFO_STREAM(main_subscriber_.size());
}

MainMonitor::MainMonitor(std::string config_file): cpu_monitor_() {
    ROS_INFO("Constructor Monitor");
    bool statistics_flags = true;
    config_file_ = config_file;
    std::string path = ros::package::getPath("gr_fdd");
    YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_file_).c_str());
    
    const YAML::Node& signals_ids = config_yaml["signals"];
    int id = 0;
 
    //for (int i=0; i< topic_names.size(); ++i){
    for (YAML::const_iterator a= signals_ids.begin(); a != signals_ids.end(); ++a){
    //for (YAML::Node::iterator it = topic_names.begin(); it != topic_names.end(); ++it){
        std::string name = a->first.as<std::string>();
        YAML::Node config = a->second;
        double window_size = config["window_size"].as<double>();
        double max_delay = config["max_delay"].as<double>();
        double max_diff = config["max_diff"].as<double>();
        double min_diff = config["min_diff"].as<double>();
        int samples = config["samples"].as<int>();
        ROS_INFO_STREAM("Signal to monitor "<< name);
        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::in_cb, this, _1, id, name) ;
        data_containers_.emplace_back(name, statistics_flags, samples, window_size,max_delay,max_diff,min_diff);
        main_subscriber_.push_back( ros::Subscriber(node.subscribe(name, 10, callback)));
        ++id;
        //statistics_flags = !statistics_flags;
    }
    recovery_executor_ = new gr_fdd::RecoveryExecutor(config_yaml["recovery"]);
    ros::NodeHandle nh;
    timer_ = nh.createTimer(ros::Duration(0.3), &MainMonitor::detect_faults,this);
    monitor_status_pub_ = nh.advertise<std_msgs::Int8>("monitor_status", 10);
    monitor_diagnostic_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics_agg", 10);

    ros::spin();
}


std::string MainMonitor::isolate_components(std::list<std::string> error_topics){
    error_topics.sort();
    std::string path = ros::package::getPath("gr_fdd");
    YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_file_).c_str());
    const YAML::Node& faults = config_yaml["faults"];
    int counter = 0;
    
    std::string error_description = "";
    ROS_INFO_STREAM(error_topics.size() << " Errors reported");
 
    for (YAML::const_iterator fault= faults.begin(); fault != faults.end(); ++fault){
        std::string fault_id = fault->first.as<std::string>();
        //std::string v = a->second.as<std::string>();
        std::list<std::string> signals = fault->second.as<std::list<std::string> >();
        //std::cout << n << v <<std::endl;
        /*
        for (auto i = v.begin(); i!= v.end(); ++i){
           std::cout << *i << std::endl;
        }
        */
        
        signals.sort();
        counter = 0;
        for (auto i1 = signals.begin(); i1!= signals.end(); ++i1){
            for (auto i2 = error_topics.begin(); i2!= error_topics.end(); ++i2){
                if (i1->compare(*i2)==0){
                    ++counter;
                }
            }
        }
        
        if (counter == signals.size()){
            ROS_WARN_STREAM ("Suggested action: " << recovery_executor_->getRecoveryStrategy(fault_id));
            ROS_ERROR_STREAM("Error Message " << fault_id);
            error_description += fault_id + "::::";

        }
   }
    
    return error_description.empty() ? "Unknown error" : error_description;
}

MainMonitor::MainMonitor(const MainMonitor& orig) {
}

MainMonitor::~MainMonitor() {
}

void MainMonitor::detect_faults(const ros::TimerEvent&){
    
    diagnostic_msgs::DiagnosticArray diagnostic_msg;
    diagnostic_msg.header.stamp = ros::Time::now();
    std::map<std::string, std::string> map_messages;

    
    diagnostic_msgs::DiagnosticStatus diagnostic_status;
    diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnostic_status.name = "navigation_components";
    diagnostic_status.message = "Expected Performance";
    diagnostic_status.hardware_id = "navigation_fdd";
    
    std::list <std::string> detected_errors;
    std_msgs::Int8 status;
    status.data = 0;

    for (std::vector<gr_fdd::DataContainer>::iterator it=data_containers_.begin(); it != data_containers_.end(); ++it){
        //std::lock_guard<std::mutex> lk(it->mtx_);
        //ROS_INFO_STREAM("Check Monitor " << it->getId());
        if (it->check()){
            status.data = 100;
            detected_errors.push_back(it->getId());
            diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
            //std::cout << it->getCurrentStatus() << map_messages[it->getCurrentStatus()] << std::endl;
        }

        map_messages[it->getCurrentStatus()]+= it->getId();
        
        //it->reset();
        
        //it->unlock();   
    }

    //Filling messages
    diagnostic_msgs::KeyValue key_value;
        
    for (std::map<std::string, std::string>::iterator it=map_messages.begin(); it!=map_messages.end(); ++it){
        key_value.key = it->first;
        key_value.value = it->second;
        diagnostic_status.values.push_back(key_value);
    }
    
    readStatsCPU();
    key_value.key = "CPU_usage";
    key_value.value = std::to_string(cpu_monitor_.getUsage());
    diagnostic_status.values.push_back(key_value);

    bool error_detected = false;
 
    if (detected_errors.size()>0){
           diagnostic_status.message = isolate_components(detected_errors);
           error_detected = true;
    }

        
    diagnostic_msg.status.push_back(diagnostic_status);
    monitor_status_pub_.publish(status);
    monitor_diagnostic_pub_.publish(diagnostic_msg);
  
    if (error_detected)
        usleep(500); //just for visualization purposes
}

double MainMonitor::readStatsCPU(){
    std::ifstream fileStat("/proc/stat");
    std::string line;
    bool flag;
    flag = true;
    
    while(std::getline(fileStat, line)){
        // cpu stats line found
        if(!line.compare(0, 3, "cpu")){
            std::istringstream ss(line);
            std::string name;
            std::string cpu;
            ss >> name;

            for(int i = 0; i < 10; ++i){
                double d;
                ss >> d;
                cpu_monitor_.updateData(i, d);
            }

            if (flag){
                double cpu_use = cpu_monitor_.getUsage();
                if (cpu_use > 80){
                    ROS_ERROR_STREAM("CPU: " << name << " usage percentage " << cpu_use << " more than " << 80);
                }

                flag = false;
            }
        }
    }
    return 0.0;
}
