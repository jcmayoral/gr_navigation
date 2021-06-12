#include <mongodb_map_utils/mongodb_map_manager.h>

using namespace mongodb_map_utils;

MongoDBMapManager::MongoDBMapManager(): nh_{"~"}{
    //nh , collection, database
    message_store_ = new mongodb_store::MessageStoreProxy(nh_,"map_frame","message_store");
    update_server_ = nh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("update_map_frame", 
                        boost::bind(&MongoDBMapManager::update_frame_callback, this, boost::placeholders::_1,  boost::placeholders::_2));
    getMapFrame();
}

bool MongoDBMapManager::update_frame_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
    ROS_INFO_STREAM("Update frame service called");
    storeMessage();
    return true;
}

void MongoDBMapManager::publishStuff(){
    tf_frame_publisher_->publishTfTransform();
}

void MongoDBMapManager::storeMessage(){
    ROS_INFO("Waiting for GPS topic");
    boost::shared_ptr<sensor_msgs::NavSatFix const> gps_msg;
    gps_msg =  ros::topic::waitForMessage<sensor_msgs::NavSatFix>("fix");
    ROS_INFO("GPS topic received");
    std::string name("reference");
    //check if exists
    std::vector< boost::shared_ptr<sensor_msgs::NavSatFix> > results;
    if(message_store_->queryNamed<sensor_msgs::NavSatFix>(name, results)) {
        message_store_->updateNamed(name, *gps_msg);
        std::cout<<"Pose \""<<name<<"\" updated"<<std::endl;
    }
    else{
        std::string id(message_store_->insertNamed(name, *gps_msg));
        std::cout<<"Pose \""<<name<<"\" inserted with id "<<id<<std::endl;
    }
    getMapFrame();
}

void MongoDBMapManager::getMapFrame(){
    std::vector< boost::shared_ptr<sensor_msgs::NavSatFix> > results;
    //sensor_msgs::NavSatFix fix;

    //Get it back, by default get one
    std::string name("reference");
    if(message_store_->queryNamed<sensor_msgs::NavSatFix>(name, results)) {
        //BOOST_FOREACH( boost::shared_ptr< sensor_msgs::NavSatFix> fix,  results){
        BOOST_FOREACH( boost::shared_ptr< sensor_msgs::NavSatFix> fix,  results){
        //        ROS_INFO_STREAM("Got by name: " << *fix);
        tf_frame_publisher_ = boost::make_shared<TfFramePublisher>(true, *fix);
        }

    }
    else{
        storeMessage();
    }
    results.clear();
}


MongoDBMapManager::~MongoDBMapManager(){
    
}