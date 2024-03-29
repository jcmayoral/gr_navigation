#include <gr_map_utils/osm_to_metric_converter.h>

using namespace grid_map;

namespace gr_map_utils{
    Osm2MetricMap::Osm2MetricMap(ros::NodeHandle nh): nh_(nh), osm_map_(), distance_to_origin_(100),tf2_listener_(tf_buffer_), gridmap_({""}), is_ready_(false){
        //TO BE TESTED
        gridmap_.setFrameId("map");
        //TODO Create a setup Gridmap function
        gridmap_.setGeometry(Length(100, 100), 0.05);
        gridmap_.add("example", Matrix::Random(gridmap_.getSize()(0), gridmap_.getSize()(1)));

        gridmap_pub_ =  nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);


        gr_tf_publisher_ = new TfFramePublisher();
        message_store_ = new mongodb_store::MessageStoreProxy(nh,"topological_maps");
        is_map_received_ = false;
        topological_map_pub_ = nh_.advertise<navigation_msgs::TopologicalMap>("topological_map", 1, true);
        osm_map_sub_ = nh_.subscribe("visualization_marker_array",10, &Osm2MetricMap::osm_map_cb, this);
        dyn_server_cb_ = boost::bind(&Osm2MetricMap::dyn_reconfigureCB, this, _1, _2);
      	dyn_server_.setCallback(dyn_server_cb_);
    }

    Osm2MetricMap::~Osm2MetricMap(){

    }

    //TODO make_pair (Remember project just a proof of concept (Meeting 2020))
    void Osm2MetricMap::fillPolygon(std::vector<double>x, std::vector<double> y){
        grid_map::Polygon polygon;
        polygon.setFrameId(gridmap_.getFrameId());
        //Load footprint as a list of pair x y
        //add each tuple as a vertex
        std::cout << "SIZE "<< x.size() << std::endl;

        std::vector<std::pair<double, double>> target;
        target.reserve(x.size());
        std::transform(x.begin(), x.end(), y.begin(), std::back_inserter(target),
               [](double a, double b) { return std::make_pair(a, b); });

        for (auto& m : target){
            std::cout << m.first << ":::: "<< m.second << std::endl;
            polygon.addVertex(grid_map::Position(m.first, m.second));
        }
        //assign values in the gridmap
        for (grid_map::PolygonIterator iterator(gridmap_,polygon); !iterator.isPastEnd(); ++iterator) {
            gridmap_.at("example", *iterator) = 255;
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
        boost::shared_ptr<visualization_msgs::MarkerArray const> osm_map;
        osm_map =  ros::topic::waitForMessage<visualization_msgs::MarkerArray>("visualization_marker_array", ros::Duration(3.0));
        if (osm_map != NULL){
            osm_map_ = *osm_map;
            ROS_INFO("OSM Map gotten");
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

        std::string needle = "buildings_osm";
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
            ROS_ERROR_STREAM("ERROR->(()) TO FILTER MAP :::-> " << marker);


            hack =  &marker.ns[0u];

            if (true){//gr_tf_publisher_->getEuclideanDistanceToOrigin(it->pose.position.x , it->pose.position.y) > 10000){//osm server has some issues with frames
            //if (true){
                in.header.frame_id = "world";
                in.pose.position.x = it->pose.position.x;
                in.pose.position.y = it->pose.position.y;
                in.pose.orientation.w =1.0;
                ROS_WARN_STREAM(in);
                to_map_transform = tf_buffer_.lookupTransform("map", "world", ros::Time(0), ros::Duration(1.0) );
                tf2::doTransform(in, out, to_map_transform);
                marker.header = out.header;
                marker.pose = out.pose; //visualization_msgs "OSM Map"
            }

            bool marker_of_interest = false;

            //MarkerArray
            if (std::strcmp(needle.c_str(), hack.c_str()) == 0){// if building then pass to static map
                marker_of_interest = true;
                //std::cout << it->points.size() << std::endl;
            }


            x.clear();
            y.clear();

            for (std::vector<geometry_msgs::Point>::iterator it_point = it->points.begin() ; it_point != it->points.end(); ++it_point){
                //osm server has some issues with frames some points come on world frame so quick fix(distance to origin > 10000) is implemented but must be changed

                if (true){//gr_tf_publisher_->getEuclideanDistanceToOrigin(it_point->x, it_point->y) > 10000){//osm server has some issues with frames
                    in.header.frame_id = "world";
                    in.pose.position.x = it_point->x;
                    in.pose.position.y = it_point->y;
                    to_map_transform = tf_buffer_.lookupTransform("map", "world", ros::Time(0), ros::Duration(1.0) );
                    tf2::doTransform(in, out, to_map_transform);
                    ROS_ERROR("WHYYYYYYYYYYYYYYYY");
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


        auto ba = boundaries_x.begin();
        auto bb = boundaries_y.begin();

        for (;ba!=boundaries_x.end();ba++,bb++){
            std::cout << "BOUNDARIES "<< *ba << " :::: " << *bb << std::endl;
        }

        auto minx = *std::min_element(std::begin(boundaries_x), std::end(boundaries_x));
        auto maxx = *std::max_element(std::begin(boundaries_x), std::end(boundaries_x));
        auto miny = *std::min_element(std::begin(boundaries_y), std::end(boundaries_y));
        auto maxy = *std::max_element(std::begin(boundaries_y), std::end(boundaries_y));

        std::cout << "RANGE X" << maxx - minx << std::endl;
        std::cout << "RANGE Y" << maxy - miny << std::endl;

        gridmap_.setGeometry(Length(std::fabs(maxx-minx),std::fabs(maxy - miny)), 0.05);
        //boundaries

        float ox,oy;
        gr_tf_publisher_->getTf(ox,oy);
        //ox -= (maxx-minx)/2;
        //oy -= (maxy-miny)/2;
        grid_map::Position center;
        center(0) = 0;//ox;
        center(1) = 0;//oy;
        gridmap_.setPosition(center);

        //Filling Polygons
        auto ia = testx.begin();
        auto ib = testy.begin();

        for (;ia!=testx.end();ia++,ib++){
            //std::cout << "filling something:: " << ia->size() << std::endl;
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
            std::cout << "HERE"<<std::endl;
            GridMapRosConverter::toOccupancyGrid(gridmap_,"example", 0.0, 255.0,grid_);
            //ROS_INFO_STREAM("MAP INfO " << grid.info);
            gridmap_pub_.publish(grid_);
        }
    }

}
