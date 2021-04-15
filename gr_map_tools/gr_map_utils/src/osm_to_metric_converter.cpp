#include <gr_map_utils/osm_to_metric_converter.h>

using namespace grid_map;

namespace gr_map_utils{
    Osm2MetricMap::Osm2MetricMap(ros::NodeHandle nh, std::string config_file):
        nh_(nh), osm_map_(), distance_to_origin_(100),tf2_listener_(tf_buffer_), gridmap_({""}), is_ready_(false){
        std::cout << "Config file " << config_file << std::endl;
        YAML::Node config = YAML::LoadFile(config_file);
        needle_ = config["needle"].as<std::string>();
        type_ = config["type"].as<std::string>();
        map_frame_ = config["map_frame"].as<std::string>();

        //TO BE TESTED
        in_topic_ = config["topic"].as<std::string>();
        std::string map_topic = config["map_topic"].as<std::string>();
        auto size_x = config["size_x"].as<float>();
        auto size_y = config["size_y"].as<float>();

        gridmap_.setFrameId(map_frame_);
        //TODO Create a setup Gridmap function
        gridmap_.setGeometry(Length(size_x, size_y), 1);
        gridmap_.add("example", Matrix::Random(gridmap_.getSize()(0), gridmap_.getSize()(1)));

        gridmap_pub_ =  nh_.advertise<nav_msgs::OccupancyGrid>(map_topic, 1, true);


        YAML::Node tf_node = config["TF"];

        bool initialize_tf = (bool) tf_node["enable_tf"].as<int>();
        std::string origin_frame = tf_node["origin_frame"].as<std::string>();
        std::string output_frame = tf_node["output_frame"].as<std::string>();

        gr_tf_publisher_ = new TfFramePublisher(initialize_tf, origin_frame, output_frame);
        message_store_ = new mongodb_store::MessageStoreProxy(nh,"topological_maps222");
        is_map_received_ = false;
        topological_map_pub_ = nh_.advertise<navigation_msgs::TopologicalMap>("topological_map2", 1, true);
        osm_map_sub_ = nh_.subscribe(in_topic_,10, &Osm2MetricMap::osm_map_cb, this);
        dyn_server_cb_ = boost::bind(&Osm2MetricMap::dyn_reconfigureCB, this, _1, _2);
      	dyn_server_.setCallback(dyn_server_cb_);
    }

    Osm2MetricMap::~Osm2MetricMap(){

    }

    //TODO make_pair (Remember project just a proof of concept (Meeting 2020))
    void Osm2MetricMap::fillPolygon(std::vector<double>x, std::vector<double> y){
        //Load footprint as a list of pair x y
        //add each tuple as a vertex
        //std::cout << "SIZE "<< x.size() << std::endl;

        std::vector<std::pair<double, double>> target;
        target.reserve(x.size());
        std::transform(x.begin(), x.end(), y.begin(), std::back_inserter(target),
               [](double a, double b) { return std::make_pair(a, b); });

        std::cout << "receiving " << x.size() << " , " << y.size() <<std::endl;

        if (type_.compare("line") ==0){
          grid_map::Index start;
          grid_map::Position startPose;
          grid_map::Index end;
          grid_map::Position endPose;

          for (auto& m : target){
            auto index = &m - &target[0];
            startPose(0) = target[0].first;
            startPose(1) = target[0].second;
            gridmap_.getIndex(startPose, start);

            endPose(0) = m.first;
            endPose(1) = m.second;
            gridmap_.getIndex(endPose, end);

            for (grid_map::LineIterator iterator(gridmap_, start, end);
                !iterator.isPastEnd(); ++iterator) {
              //std::cout << "polygon " << in_topic_ << std::endl;
              gridmap_.at("example", *iterator) = 64;
            }
          }
          return;
        }

        //POLYGON
        grid_map::Polygon polygon;
        polygon.setFrameId(gridmap_.getFrameId());

        //assign values in the gridmap
        for (auto& m : target){
            polygon.addVertex(grid_map::Position(m.first, m.second));
        }

        for (grid_map::PolygonIterator iterator(gridmap_,polygon); !iterator.isPastEnd(); ++iterator) {
            std::cout << "polygon " << in_topic_ << std::endl;
            gridmap_.at("example", *iterator) = 64;
        }
    }

    void Osm2MetricMap::dyn_reconfigureCB(OSMMapConverterConfig &config, uint32_t level){
        ROS_INFO("aaaa");
        if (!is_map_received_){
            ROS_WARN("Map not received");
            return;
        }
        ROS_INFO("updating Map");
        //Update values
        distance_to_origin_ = config.distance_to_origin;

        transformMap();
    }


    bool Osm2MetricMap::storeMap(){
        std::string name("osm_to_metric_map");
        //TODO find a way to store it
        //std::string id(message_store_->insertNamed(name, gridmap_));
        //message_store_->updateID(id, gridmap_);
        return true;
    }

    bool Osm2MetricMap::getMapFromDatabase(){
        return false;
    }

    bool Osm2MetricMap::getMapFromTopic(){
        std::unique_lock<std::mutex> lk(mutex_);
        std::cout << "IN TOPIC "<< in_topic_ << std::endl;

        boost::shared_ptr<visualization_msgs::MarkerArray const> osm_map;
        osm_map =  ros::topic::waitForMessage<visualization_msgs::MarkerArray>(in_topic_, ros::Duration(10.0));

        if (osm_map != NULL){
            osm_map_ = *osm_map;
            ROS_INFO_STREAM("OSM Map gotten topic " << in_topic_);
            return true;
            //ROS_INFO_STREAM("Got by topic: " << topological_map_);
        }
        return false;
    }

    bool Osm2MetricMap::getMapFromService(){
        return false;
    }

    void Osm2MetricMap::transformMap(){
        gr_tf_publisher_->publishTfTransform();
        int count = 0;

        geometry_msgs::TransformStamped to_map_transform; // My frames are named "base_link" and "leap_motion"
        geometry_msgs::PoseStamped out;
        geometry_msgs::PoseStamped in;
        in.pose.orientation.w = 1.0;

        std::string hack = "others_osm";
        std::string boundshack = "bounds_osm";

        std::vector<double> x;
        std::vector<double> y;

        std::vector<std::vector<double>> testx;
        std::vector<std::vector<double>> testy;

        std::vector<double> boundaries_x;
        std::vector<double> boundaries_y;

        for (std::vector<visualization_msgs::Marker>::iterator it = osm_map_.markers.begin(); it != osm_map_.markers.end(); ++it){
            if (it->type != 4 && it->type != 5){//Just go for lines
                ROS_DEBUG_STREAM("IGNORE "<< it->type);
                continue;
            }

            x.clear();
            y.clear();
            visualization_msgs::Marker marker(*it);
            //ROS_ERROR_STREAM("ERROR->(()) TO FILTER MAP :::-> " << marker);

            hack =  &marker.ns[0u];

            if (it->header.frame_id.compare(map_frame_)!=0){//gr_tf_publisher_->getEuclideanDistanceToOrigin(it->pose.position.x , it->pose.position.y) > 10000){//osm server has some issues with frames
            //if (true){
                in.header.frame_id = it->header.frame_id;
                in.pose.position.x = it->pose.position.x;
                in.pose.position.y = it->pose.position.y;
                in.pose.orientation.w =1.0;
                to_map_transform = tf_buffer_.lookupTransform(map_frame_,it->header.frame_id, ros::Time(0), ros::Duration(1.0) );
                tf2::doTransform(in, out, to_map_transform);
                marker.header = out.header;
                marker.pose = out.pose; //visualization_msgs "OSM Map"
            }

            bool marker_of_interest = false;

            //MarkerArray
            if (std::strcmp(needle_.c_str(), hack.c_str()) == 0){// if building then pass to static map
                marker_of_interest = true;
                //std::cout << it->points.size() << std::endl;
            }

            x.clear();
            y.clear();

            for (std::vector<geometry_msgs::Point>::iterator it_point = it->points.begin() ; it_point != it->points.end(); ++it_point){
                //osm server has some issues with frames some points come on world frame so quick fix(distance to origin > 10000) is implemented but must be changed

                if (it->header.frame_id.compare(map_frame_)!=0){//gr_tf_publisher_->getEuclideanDistanceToOrigin(it_point->x, it_point->y) > 10000){//osm server has some issues with frames
                    in.header.frame_id = "world";
                    in.pose.position.x = it_point->x;
                    in.pose.position.y = it_point->y;
                    to_map_transform = tf_buffer_.lookupTransform(map_frame_, it->header.frame_id, ros::Time(0), ros::Duration(1.0) );
                    tf2::doTransform(in, out, to_map_transform);
                    it_point->x = out.pose.position.x;
                    it_point->y = out.pose.position.y;
                }

                //Take Borders
                if (std::strcmp(hack.c_str(), boundshack.c_str()) == 0){
                    //TYpe 4 Line Strip
                    boundaries_x.push_back(it_point->x);
                    boundaries_y.push_back(it_point->y);
                    continue;
                }

                if (!marker_of_interest){
                    continue;
                }

                //Take Buildings
                x.push_back(it_point->x);
                y.push_back(it_point->y);

            }

            if (x.size()>1){
                testx.push_back(x);
                testy.push_back(y);
            }
        }


        std::cout << "boundaries " << in_topic_ << std::endl;
        std::cout << boundaries_x.size() << std::endl;
        std::cout << "x  " << x.size() << std::endl;
        std::cout << "testx  " << testx.size() << std::endl;

        float ox,oy, minx, miny;
        if (boundaries_x.size() > 1){

          auto ba = boundaries_x.begin();
          auto bb = boundaries_y.begin();

          /*
          for (;ba!=boundaries_x.end();ba++,bb++){
              std::cout << "BOUNDARIES "<< *ba << " :::: " << *bb << std::endl;
          }
          */

          minx = *std::min_element(std::begin(boundaries_x), std::end(boundaries_x));
          auto maxx = *std::max_element(std::begin(boundaries_x), std::end(boundaries_x));
          miny = *std::min_element(std::begin(boundaries_y), std::end(boundaries_y));
          auto maxy = *std::max_element(std::begin(boundaries_y), std::end(boundaries_y));

          //std::cout << "RANGE X" << maxx - minx << std::endl;
          //std::cout << "RANGE Y" << maxy - miny << std::endl;

          //gridmap_.setGeometry(Length(std::fabs(maxx-minx),std::fabs(maxy - miny)), 0.05);
          //boundaries

          gr_tf_publisher_->getTf(ox,oy);
          ox = (maxx-minx)/2;
          oy = (maxy-miny)/2;
        }
        else{
          std::cout << "else " << in_topic_ << std::endl;
          ox = 0;
          oy = 0;
          minx = 0.0;
          miny = 0.0;
          //gridmap_.setGeometry(Length(200,200), 2.5);
        }
        grid_map::Position center;
        center(0) = ox+minx;
        center(1) = oy+miny;
        gridmap_.setPosition(center);

        //Filling Polygons
        auto ia = testx.begin();
        auto ib = testy.begin();

        for (;ia!=testx.end();ia++,ib++){
          std::cout << "Calling polygon " << std::endl;
            fillPolygon(*ia,*ib);
        }
        is_ready_ = true;
    }

    void Osm2MetricMap::osm_map_cb(const visualization_msgs::MarkerArray::ConstPtr& map){
        osm_map_ = *map;
    }

    void Osm2MetricMap::publishMaps(){
        gr_tf_publisher_->publishTfTransform();
        //TO BE TESTED
        //Signature of function
        //GridMapRosConverter::toOccupancyGrid(const grid_map::GridMap& gridMap,const std::string& layer, float dataMin, float dataMax,nav_msgs::OccupancyGrid& occupancyGrid);
        //TODO set proper dataMin/dataMax values
        // GridMap GridMap::getTransformedMap(const Eigen::Isometry3d& transform, const std::string& heightLayerName, const std::string& newFrameId,const double sampleRatio)
        if (is_ready_){
            GridMapRosConverter::toOccupancyGrid(gridmap_,"example", 0.0, 255.0,grid_);
            //ROS_INFO_STREAM("MAP INfO " << grid.info);
            gridmap_pub_.publish(grid_);
        }
    }

}
