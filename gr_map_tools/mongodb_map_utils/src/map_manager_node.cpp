#include <mongodb_map_utils/mongodb_map_manager.h>

using namespace mongodb_map_utils;

int main(int argc, char** argv){
    ros::init(argc, argv, "mongodb_map_utils");
    MongoDBMapManager manager;
    ros::Rate loop_rate(10);

     while (ros::ok()){
        loop_rate.sleep();
        manager.publishStuff();
        ros::spinOnce();
    }

}
