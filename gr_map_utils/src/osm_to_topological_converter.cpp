#include <gr_map_utils/osm_to_topological_converter.h>

namespace gr_map_utils{

    Osm2TopologicalMap::Osm2TopologicalMap(ros::NodeHandle nh){
        message_store_ = new mongodb_store::MessageStoreProxy(nh);
        //gr_tf_publisher_ = new TfFramePublisher();
        //nh.subscriber("visualization_marker_arrat", )
    }

    Osm2TopologicalMap::~Osm2TopologicalMap(){

    }

    bool Osm2TopologicalMap::storeMap(){
        return false;
    }

    bool Osm2TopologicalMap::getMap(){
        return false;
    }

}
