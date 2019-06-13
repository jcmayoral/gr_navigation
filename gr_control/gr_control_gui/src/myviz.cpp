/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "myviz.h"


using namespace gr_control_gui;
// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent ), marker_array_(), nh_(), robot_radius_(2.0), current_row_(0), terrain_x_(1.0), terrain_y_(1.0)
{
  map_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("temporal_topological_map", 1 );
  region_publisher_ = nh_.advertise<visualization_msgs::Marker>("region", 1 );
  reset_publisher_ = nh_.advertise<std_msgs::Time>("update_map", 1);
  //collection and database as arguments to messageStoreProxy
  message_store_ = new mongodb_store::MessageStoreProxy(nh_,"topological_maps","message_store");

  // Construct and lay out labels and slider controls.
  QLabel* width_label = new QLabel( "Y Terrain" );
  QSlider* width_slider = new QSlider( Qt::Horizontal );
  width_slider->setMinimum( 1.00 );
  width_slider->setMaximum( 100.0 );

  QLabel* height_label = new QLabel( "X Terrain" );
  QSlider* height_slider = new QSlider( Qt::Horizontal );
  height_slider->setMinimum( 1.0 );
  height_slider->setMaximum( 100.0 );

  QLabel* column_label = new QLabel("Desired Row");
  QSpinBox* column_spinbox = new QSpinBox;
  column_spinbox->setRange(0, 100);
  column_spinbox->setSingleStep(1);
  column_spinbox->setValue(0);

  QPushButton* save_topological_map = new QPushButton ("Store Map");
  QPushButton* execute_map = new QPushButton ("Execute Map");

  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( width_label, 0, 0 );
  controls_layout->addWidget( width_slider, 0, 1 );
  controls_layout->addWidget( height_label, 1, 0 );
  controls_layout->addWidget( height_slider, 1, 1 );
  controls_layout->addWidget( column_label, 2, 0 );
  controls_layout->addWidget( column_spinbox, 2, 1 );
  controls_layout->addWidget( save_topological_map, 3, 0 );
  controls_layout->addWidget( execute_map, 3, 1 );

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  connect( width_slider, SIGNAL( valueChanged( int )), this, SLOT( setTerrainY(  int )));
  connect( height_slider, SIGNAL( valueChanged( int )), this, SLOT( setTerrainX(  int)));
  connect( execute_map, SIGNAL( released( )), this, SLOT( executeTopoMap( )));
  connect( save_topological_map, SIGNAL( released( )), this, SLOT( saveMap( )));
  connect( column_spinbox, SIGNAL(valueChanged(int)), this, SLOT(setDesiredRow(int)));

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  // Create Subscribers
  rviz::Display* marker_display;
  marker_display = manager_->createDisplay( "rviz/MarkerArray", "topological_map", true );
  ROS_ASSERT( marker_display != NULL );
  marker_display->subProp( "Marker Topic" )->setValue("temporal_topological_map");

  rviz::Display* region_marker;
  region_marker = manager_->createDisplay( "rviz/Marker", "region_marker", true );
  ROS_ASSERT( region_marker != NULL );
  region_marker->subProp( "Marker Topic" )->setValue("region");

  rviz::Display* proximity_marker;
  proximity_marker = manager_->createDisplay( "rviz/Marker", "proximity_marker", true );
  ROS_ASSERT( proximity_marker != NULL );
  proximity_marker->subProp( "Marker Topic" )->setValue("proximity_visualization");

  rviz::Display* robot_display;
  robot_display = manager_->createDisplay( "rviz/RobotModel", "thorvald", true );
  ROS_ASSERT( robot_display != NULL );
  //robot_display->subProp( "Marker Topic" )->setValue("region");



  // Initialize the slider values.
  height_slider->setValue( 10.0 );
  width_slider->setValue( 10.0 );

  manager_->setFixedFrame("map");
  manager_->initialize();
  manager_->startUpdate();
  update_client_ = nh_.serviceClient<gr_map_utils::UpdateMap>("update_metric_map");

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

void MyViz::publishRegion(){
  visualization_msgs::Marker region;
  region.header.frame_id = "map";
  region.ns = "region";
  region.id = 20001;
  region.type = visualization_msgs::Marker::LINE_STRIP;
  region.action = visualization_msgs::Marker::DELETE;

  region_publisher_.publish(region);
  region.action = visualization_msgs::Marker::ADD;

  region.scale.x = 0.1;
  region.scale.y = 0.1;
  region.scale.z = 0.1;
  region.color.r = 0.0;
  region.color.g = 1.0;
  region.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = -robot_radius_/2;
  p.y = -robot_radius_/2;
  p.z = 0.0;
  region.points.push_back(p);

  p.x = terrain_x_ - robot_radius_/2;
  p.y = - robot_radius_/2;
  p.z = 0.0;
  region.points.push_back(p);

  p.x = terrain_x_ - robot_radius_/2;
  p.y = terrain_y_ - robot_radius_/2;
  p.z = 0.0;
  region.points.push_back(p);

  p.x = -robot_radius_/2;
  p.y = terrain_y_ - robot_radius_/2;
  p.z = 0.0;
  region.points.push_back(p);

  p.x = -robot_radius_/2;
  p.y = -robot_radius_/2;
  p.z = 0.0;
  region.points.push_back(p);

  region_publisher_.publish(region);

}

void MyViz::setTerrainY( int value){
  terrain_y_ = value;
  y_cells_ = ceil(value/1);
  //visualizeMap();
}

void MyViz::setTerrainX( int value ){
  terrain_x_ = value;
  x_cells_ = ceil(value/1);
  //visualizeMap();
}

void MyViz::visualizeMap(){
  //Node Creation
  node_map_.clear();
  visualization_msgs::Marker temporal_marker;

  marker_array_.markers.clear();
  edges_.clear();

  //For now this fields are constants FOR NODES
  temporal_marker.header.frame_id="map";
  temporal_marker.header.stamp = ros::Time::now();
  temporal_marker.ns = "nodes"; //TODO maybe add segmentation layers
  temporal_marker.type = visualization_msgs::Marker::CYLINDER;

  //DELETE PREVIOUS
  temporal_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array_.markers.push_back(temporal_marker);
  map_publisher_.publish(marker_array_);

  //Create New Nodes
  temporal_marker.action = visualization_msgs::Marker::ADD;
  temporal_marker.scale.x = robot_radius_/3;//divided by three just o see edges
  temporal_marker.scale.y = robot_radius_/3;
  temporal_marker.scale.z = 0.05;
  temporal_marker.color.r = 1.0;
  temporal_marker.color.a = 1.0;

  temporal_marker.pose.orientation.w = 1.0;

  //For edges
  geometry_msgs::Point temporal_point;
  visualization_msgs::Marker temporal_edges;

  temporal_edges.header.frame_id="map";
  temporal_edges.header.stamp = ros::Time::now();
  temporal_edges.ns = "edges"; //TODO maybe add segmentation layers
  temporal_edges.type = visualization_msgs::Marker::LINE_LIST;
  temporal_edges.action = visualization_msgs::Marker::ADD;
  temporal_edges.scale.x = 0.1;
  temporal_edges.scale.y = 0.1;
  temporal_edges.scale.z = 0.5;
  temporal_edges.color.r = 1.0;
  temporal_edges.color.g = 1.0;
  temporal_edges.color.a = 1.0;
  temporal_edges.pose.orientation.w = 1.0;


  std::vector<std::pair<float,float> > vector;

  map_utils_->calculateCenters(vector,  x_cells_, y_cells_, 1.0, 1.0);
  std::cout << "cells " << x_cells_ << " , " << y_cells_ << std::endl;
  int id, index_1, index_2 = 0;
  int col;

  int min_index = (current_row_*y_cells_);
  int max_index = (current_row_*y_cells_) + y_cells_;
  std::cout << "indices  " << min_index << " , " << max_index << std::endl;
  double yaw =(current_row_%2) ? -1.57 : 1.57;


  for( auto id = min_index; id< max_index; ++id){
    //Storing Nodes
    col = id/y_cells_;
    std::cout << "COOOOOOOOOOOL " << col << std::endl;
    temporal_marker.id = id;
    temporal_marker.pose.position.x = vector[id].first;
    temporal_marker.pose.position.y = vector[id].second;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, yaw);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat_tf, temporal_marker.pose.orientation);

    marker_array_.markers.push_back(temporal_marker);

    std::string id_str("error");
    std::string next_id_str("error");

    id_str ="node_" + std::to_string(id);
    next_id_str ="node_" + std::to_string(id+1);

    //Nasty Hack
    if (id == min_index){
      id_str = "start_node";
      if (col%2 == 1){
         id_str = "end_node";
      }
    }
    else if(id == max_index-1){
      id_str = "end_node";
      if (col%2 == 1){
         id_str = "start_node";
      }
    }
    else{
      id_str ="node_" + std::to_string(id);
    }

    if ((id+1) == min_index){
      next_id_str = "start_node";
      if (col%2 == 1){
         next_id_str = "end_node";
      }
    }
    else if(id == (max_index-2)){
      next_id_str = "end_node";
      if (col%2 == 1){
         next_id_str = "start_node";
      }
    }
    else{
      next_id_str ="node_" + std::to_string(id+1);
    }
    //end of nasty hack
    node_map_[id_str] = temporal_marker.pose;

    if (id == max_index-1){
      ROS_ERROR("AAAAAA");
      continue;
    }


    temporal_edges.id = 100+id;
    temporal_point.x = vector[id].first;
    temporal_point.y = vector[id].second;
    temporal_edges.points.push_back(temporal_point);
    temporal_point.x = vector[id+1].first;
    temporal_point.y = vector[id+1].second;
    //Marker
    temporal_edges.points.push_back(temporal_point);
    //Edges ids

    //birectional
    std::cout << id_str << next_id_str << std::endl;
    edges_.emplace_back(id_str, next_id_str);
    edges_.emplace_back(next_id_str,id_str);

    marker_array_.markers.push_back(temporal_edges);
  }

  map_publisher_.publish(marker_array_);
  publishRegion();
}

void MyViz::deleteTopoMap(std::string map_id){


    std::string id("remove_collection");
  	message_store_->deleteID(id+"--"+map_id);
   /*
   std::vector< boost::shared_ptr<strands_navigation_msgs::TopologicalNode> > results_node;

   std::vector<std::string> fields;
   fields.push_back("map");
   fields.push_back("pointset");

   std::vector<std::string> ids;
   ids.push_back(map_id);
   ids.push_back(map_id);

   if(message_store_->queryNamed<strands_navigation_msgs::TopologicalNode>(map_id, "pointset", results_node, false)) {
            strands_navigation_msgs::TopologicalNode node;
            BOOST_FOREACH( boost::shared_ptr<  strands_navigation_msgs::TopologicalNode> node,  results_node){
                ROS_DEBUG_STREAM("Got by name: " << *node);
            }
  }
  */
}

void MyViz::saveMap(){

  strands_navigation_msgs::TopologicalMap topo_map;
  strands_navigation_msgs::TopologicalNode topo_node;
  strands_navigation_msgs::Vertex vertex;

  strands_navigation_msgs::Edge edge;

  //std::string map_id("trash_map_5");
  std::string map_id("wish_map_move_base");
  deleteTopoMap(map_id);
  topo_map.map = map_id;
  topo_map.name =  map_id;
  topo_map.pointset = map_id;


  topo_node.map = map_id;
  topo_node.name = map_id;
  topo_node.pointset = map_id;
  topo_node.localise_by_topic = map_id;

  //TODO this is a hack for the python mongodb implementation
  // Just work on Jose's mongodb_store repository
  std::vector<std::string> fields;
  fields.push_back("map");
  fields.push_back("node");
  fields.push_back("pointset");
  fields.push_back("name");

  std::vector<std::string> ids;
  ids.push_back(map_id);
  ids.push_back(map_id);
  ids.push_back(map_id);
  ids.push_back(map_id);

  for (auto const & node : node_map_){
    topo_node.edges.clear();
    topo_node.verts.clear();
    topo_node.pose = node.second;
    ids[1] = node.first;
    ids[3] = node.first;
    topo_node.name = node.first;

    vertex.x = -robot_radius_/2;
    vertex.y = robot_radius_/2;
    topo_node.verts.push_back(vertex);
    vertex.x = robot_radius_/2;
    vertex.y = robot_radius_/2;

    topo_node.verts.push_back(vertex);
    vertex.x = robot_radius_/2;
    vertex.y = -robot_radius_/2;
    topo_node.verts.push_back(vertex);
    vertex.x = -robot_radius_/2;
    vertex.y = -robot_radius_/2;
    topo_node.verts.push_back(vertex);

    for (Edges & e : edges_){
      if (e.first.compare(node.first)==0){
        edge.edge_id = e.first + "_" + e.second;
        edge.node = e.second;
        edge.action = "sbpl_action";
        topo_node.edges.push_back(edge);
      }
    }

    std::string result(message_store_->insertNamed( fields, ids, topo_node));
    //std::string result(message_store_->insertNamed("pointset", map_id, topo_node));
    topo_map.nodes.push_back(topo_node);
  }


  //std::string result(message_store_->insertNamed( field, name, topo_map));
  //ROS_INFO_STREAM("Map inserted at collection " << message_store_->getCollectionName());
}

void MyViz::setDesiredRow(int row){
  current_row_ = row;
  ROS_INFO_STREAM("Desired row");
  visualizeMap();
}

void MyViz::executeTopoMap(){
  reset_publisher_.publish(std_msgs::Time());
  gr_map_utils::UpdateMap req;
  if(update_client_.call(req)){
    ROS_INFO("Client Succeded");
  }
}
