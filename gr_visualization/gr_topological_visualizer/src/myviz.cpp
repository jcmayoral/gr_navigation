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


using namespace gr_topological_visualizer;
// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent ), marker_array_(), nh_(), robot_radius_(2.0)
{
  map_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("temporal_topological_map", 1 );
  message_store_ = new mongodb_store::MessageStoreProxy(nh_,"topological_maps");

  // Construct and lay out labels and slider controls.
  QLabel* width_label = new QLabel( "Width Terrain" );
  QSlider* width_slider = new QSlider( Qt::Horizontal );
  width_slider->setMinimum( 0.00 );
  width_slider->setMaximum( 100.0 );

  QLabel* height_label = new QLabel( "Height Terrain" );
  QSlider* height_slider = new QSlider( Qt::Horizontal );
  height_slider->setMinimum( 1.0 );
  height_slider->setMaximum( 100.0 );

  QLabel* robot_radius_label = new QLabel("Robot Radius");
  QSpinBox* robot_radius_spinbox = new QSpinBox;
  robot_radius_spinbox->setRange(1, 30);
  robot_radius_spinbox->setSingleStep(1);
  robot_radius_spinbox->setValue(robot_radius_);

  QPushButton* save_topological_map = new QPushButton ("Store Map");

  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( width_label, 0, 0 );
  controls_layout->addWidget( width_slider, 0, 1 );
  controls_layout->addWidget( height_label, 1, 0 );
  controls_layout->addWidget( height_slider, 1, 1 );
  controls_layout->addWidget( robot_radius_label, 2, 0 );
  controls_layout->addWidget( robot_radius_spinbox, 2, 1 );
  controls_layout->addWidget( save_topological_map, 3, 0 );

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  connect( width_slider, SIGNAL( valueChanged( int )), this, SLOT( setTerrainWidth(  int )));
  connect( height_slider, SIGNAL( valueChanged( int )), this, SLOT( setTerrainHeight(  int)));
  connect( save_topological_map, SIGNAL( released( )), this, SLOT( saveMap( )));
  connect( robot_radius_spinbox, SIGNAL(valueChanged(int)), this, SLOT(setRobotRadius(int)));

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  // Create a MARKER
  marker_display_ = manager_->createDisplay( "rviz/MarkerArray", "topological map", true );
  ROS_ASSERT( marker_display_ != NULL );

  //subscribe to temproal topological_map
  marker_display_->subProp( "Marker Topic" )->setValue("temporal_topological_map");

  // Initialize the slider values.
  height_slider->setValue( 10.0 );
  width_slider->setValue( 10.0 );

  //marker_array_->subProp("Reference Frame")->setValue("map"); // This probably works by itself, I added the next line just in case
  //marker_array_->initialize(visualization_manager_);
  manager_->setFixedFrame("map");
  manager_->initialize();
  manager_->startUpdate();

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

void MyViz::setRobotRadius(int value){
  robot_radius_ = value;
  width_cells_ = floor(terrain_width_/robot_radius_);
  height_cells_ = floor(terrain_height_/robot_radius_);
  visualizeMap();
}

void MyViz::setTerrainWidth( int value){
  terrain_width_ = value;
  width_cells_ = floor(value/robot_radius_);
  visualizeMap();
}

void MyViz::setTerrainHeight( int value ){
  terrain_height_ = value;
  height_cells_ = floor(value/robot_radius_);
  visualizeMap();
}

void MyViz::visualizeMap(){
  //Node Creation
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
  temporal_marker.scale.x = robot_radius_/2;//divided by two just o see edges
  temporal_marker.scale.y = robot_radius_/2;
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
  temporal_edges.scale.x = 0.5;
  temporal_edges.scale.y = 0.1;
  temporal_edges.scale.z = 0.5;
  temporal_edges.color.r = 1.0;
  temporal_edges.color.g = 1.0;
  temporal_edges.color.a = 1.0;
  temporal_edges.pose.orientation.w = 1.0;  

  
  std::vector<std::pair<float,float> > vector;

  map_utils_->calculateCenters(vector,  height_cells_, width_cells_, robot_radius_, robot_radius_);

  int id, index_1, index_2 = 0;
  int col;
  int row;

  for( std::vector <std::pair <float,float> >::iterator it = vector.begin(); it != vector.end(); it++ ){
    //Storing Nodes
    id = std::distance(vector.begin(), it);
    col = round(id/height_cells_);
    row = id - col *height_cells_; 
   
    temporal_marker.id = std::distance(vector.begin(), it);
    temporal_marker.pose.position.x = it->first;
    temporal_marker.pose.position.y = it->second;
    marker_array_.markers.push_back(temporal_marker);
    std::string id_str("node_" + std::to_string(id));
    
    node_map_[id_str] = temporal_marker.pose;

  
    //Edges Creation
    if (col ==(width_cells_-1)&& row==(height_cells_-1)){//Since LINE_LIST Requires pair of points last point does not have a match
      continue; 
    }

    //is the last node of a row
    if (col==(width_cells_-1)){
      //select direction of border
      //from index_1 to index_2
      if (row%2==0){
        index_1 = col + row*width_cells_;
        temporal_edges.id = 100+index_1;
        temporal_point.x = vector[index_1].first;
        temporal_point.y = vector[index_1].second;
        temporal_edges.points.push_back(temporal_point);
        index_2 = col + (row+1)*width_cells_;
      }
      else{
        index_1 = row*width_cells_;
        temporal_edges.id = 1020+index_1;
        temporal_point.x = vector[index_1].first;
        temporal_point.y = vector[index_1].second;
        temporal_edges.points.push_back(temporal_point);
        index_2 = (row+1)*width_cells_;
      }
      temporal_edges.id = 1001+index_1;
      temporal_point.x = vector[index_2].first;
      temporal_point.y = vector[index_2].second;
      //Marker
      temporal_edges.points.push_back(temporal_point);
      //Edges ids
      edges_.emplace_back("node_"+ std::to_string(index_1),"node_" + std::to_string(index_2));
    }
    else{
      //no border nodes
      index_1 = col + row*width_cells_;
      temporal_edges.id = 100+index_1;
      temporal_point.x = vector[index_1].first;
      temporal_point.y = vector[index_1].second;
      temporal_edges.points.push_back(temporal_point);
      temporal_point.x = vector[index_1+1].first;
      temporal_point.y = vector[index_1+1].second;
      //Marker
      temporal_edges.points.push_back(temporal_point);
      //Edges ids
      edges_.emplace_back("node_"+ std::to_string(index_1),"node_" + std::to_string(index_1 + 1));
    }
    
      marker_array_.markers.push_back(temporal_edges);

  }

  map_publisher_.publish(marker_array_);
}

void MyViz::saveMap(){

  strands_navigation_msgs::TopologicalMap topo_map;
  strands_navigation_msgs::TopologicalNode topo_node;

  topo_map.map = "spare_map";
  topo_map.name = "spare_map";
  topo_map.pointset = "spare_map";

  topo_node.map = "spare_map";
  topo_node.name = "spare_map";
  topo_node.pointset = "spare_map";

  for (Edges & e  : edges_){
    std::cout << "Edge: " << e.first << " to " << e.second << std::endl;
    std::cout << "Init : " << node_map_[e.first].position.x << " and " << node_map_[e.first].position.y << std::endl;    
    std::cout << "End : " << node_map_[e.second].position.x << " and " << node_map_[e.second].position.y << std::endl;
  }
  
  for (std::vector<visualization_msgs::Marker>::iterator it = marker_array_.markers.begin(); it!= marker_array_.markers.end();it++){
    topo_node.pose = it-> pose;
    for (Edges & e : edges_){
      if (e.first.compare("node_" + std::to_string(it->id))==0){
        std::cout << it->id << " , " << e.first << std::endl;
      }
    }

    topo_map.nodes.push_back(topo_node);
  }

  std::string name = "spare_node";
  std::string field = "map";
  std::string result(message_store_->insertNamed( field, name, topo_map));
  ROS_INFO_STREAM("Map inserted at collection " << message_store_->getCollectionName());
}
