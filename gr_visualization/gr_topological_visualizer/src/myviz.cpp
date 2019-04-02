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
  : QWidget( parent ), marker_array_(), nh_()
{
  map_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("temporal_topological_map", 1 );
  
  // Construct and lay out labels and slider controls.
  QLabel* normal_cells_label = new QLabel( "Normal Nodes" );
  QSlider* normal_slider = new QSlider( Qt::Horizontal );

  QLabel* plane_cells_label = new QLabel( "Plane Nodes" );
  QSlider* plane_slider = new QSlider( Qt::Horizontal );

  normal_slider->setMinimum( 0 );
  normal_slider->setMaximum( 100 );

  plane_slider->setMinimum( 1 );
  plane_slider->setMaximum( 100 );

  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );

  QPushButton* save_topological_map = new QPushButton ("Visualize Topological Map");

  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( normal_cells_label, 0, 0 );
  controls_layout->addWidget( normal_slider, 0, 1 );
  controls_layout->addWidget( plane_cells_label, 1, 0 );
  controls_layout->addWidget( plane_slider, 1, 1 );
  controls_layout->addWidget( cell_size_label, 2, 0 );
  controls_layout->addWidget( cell_size_slider, 2, 1 );
  controls_layout->addWidget( save_topological_map, 3, 0 );


  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  connect( normal_slider, SIGNAL( valueChanged( int )), this, SLOT( setNormalNodes( int )));
  connect( plane_slider, SIGNAL( valueChanged( int )), this, SLOT( setPlaneNodes( int )));
  connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));
  connect( save_topological_map, SIGNAL( released( )), this, SLOT( visualizeMap( )));

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  // Create a Grid display.
  grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  marker_array_ = manager_->createDisplay( "rviz/MarkerArray", "topological map", true );
  ROS_ASSERT( grid_ != NULL );
  ROS_ASSERT( marker_array_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Billboards" );
  grid_->subProp( "Color" )->setValue( QColor( Qt::yellow ) );

  marker_array_->subProp( "Marker Topic" )->setValue("temporal_topological_map");

  // Initialize the slider values.
  normal_slider->setValue( 0 );
  plane_slider->setValue( 3 );
  cell_size_slider->setValue(1.0);

  //marker_array_->subProp("Reference Frame")->setValue("map"); // This probably works by itself, I added the next line just in case
  //marker_array_->initialize(visualization_manager_);
  manager_->setFixedFrame("map");
  manager_->initialize();
  manager_->startUpdate();

  std::cout <<"a"<<std::endl;

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
void MyViz::setNormalNodes( int normal_cells )
{
  normal_cells_ = normal_cells;
  if( grid_ != NULL )
  {
    //grid_->subProp("Width") -> setValue(width_cells);
    grid_->subProp("Normal Cell Count") -> setValue(normal_cells);

    //grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( width_cells / 100.0f );
  }
}

void MyViz::setPlaneNodes( int plane_cells )
{
  plane_cells_ = plane_cells;

  if( grid_ != NULL )
  {
    grid_->subProp("Plane Cell Count") -> setValue(plane_cells);
    //grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( height_cells / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  cell_size_percent_ = cell_size_percent/10.0f;

  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}

void MyViz::visualizeMap(){
  std::cout << "IN"<< std::endl;

  //Node Creation
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker temporal_marker;

  //For now this fields are constants FOR NODES
  temporal_marker.header.frame_id="map";
  temporal_marker.header.stamp = ros::Time::now();
  temporal_marker.ns = "topological_map"; //TODO maybe add segmentation layers
  temporal_marker.type = visualization_msgs::Marker::CYLINDER;
  temporal_marker.action = visualization_msgs::Marker::ADD;
  temporal_marker.scale.x = 1.0;
  temporal_marker.scale.y = 1.0;
  temporal_marker.color.r = 1.0;
  temporal_marker.color.a = 1.0;

  temporal_marker.pose.orientation.w = 1.0;  

  std::vector<std::pair<float,float> > vector;

  map_utils_->calculateCenters(vector, plane_cells_, normal_cells_, cell_size_percent_);

  for( std::vector <std::pair <float,float> >::iterator it = vector.begin(); it != vector.end(); it++ ){
    
    std::cout <<std::distance(vector.begin(), it) << std::endl;
    temporal_marker.id = std::distance(vector.begin(), it);
    temporal_marker.pose.position.x = it->first;
    temporal_marker.pose.position.y = it->second;
    marker_array.markers.push_back(temporal_marker);
    std::printf("center %f %f \n",it->first,it->second);
  }

  //map_publisher_.publish(marker_array);

  //For edges
  geometry_msgs::Point temporal_point;
  visualization_msgs::Marker temporal_edges;

  temporal_edges.header.frame_id="map";
  temporal_edges.header.stamp = ros::Time::now();
  temporal_edges.ns = "topological_map"; //TODO maybe add segmentation layers
  temporal_edges.type = visualization_msgs::Marker::LINE_LIST;
  temporal_edges.action = visualization_msgs::Marker::ADD;
  temporal_edges.scale.x = 0.3;
  temporal_edges.color.g = 1.0;
  temporal_edges.color.a = 1.0;

  temporal_edges.pose.orientation.w = 1.0;  

  int index;
    


  for (int i =0; i <plane_cells_;i++){
    for (int j =0; j <plane_cells_;j++){
      if (j==(plane_cells_-1)&& i==(plane_cells_-1)){
        continue;
      }

      if (j==(plane_cells_-1)){
        if (i%2==0){
          index = j + i*plane_cells_;
          temporal_edges.id = 100+index;
          temporal_point.x = vector[index].first;
          temporal_point.y = vector[index].second;
          temporal_edges.points.push_back(temporal_point);
          std::cout << "indexes " << index << std::endl;
          std::cout << "even " << i <<std::endl;
          index = j + (i+1)*plane_cells_;
          std::cout << "indexes " << index << std::endl;
        }
        else{
         index = i*plane_cells_;
         temporal_edges.id = 1020+index;
         temporal_point.x = vector[index].first;
         temporal_point.y = vector[index].second;
         temporal_edges.points.push_back(temporal_point);
         std::cout << "indexes " << index << std::endl;
         std::cout << "odd " << i <<std::endl;
         index = (i+1)*plane_cells_;
         std::cout << "indexes " << index << std::endl;
        }
        temporal_edges.id = 1001+index;
        temporal_point.x = vector[index].first;
        temporal_point.y = vector[index].second;
        temporal_edges.points.push_back(temporal_point);
      }
      else{
        index = j + i*plane_cells_;
        temporal_edges.id = 100+index;
        temporal_point.x = vector[index].first;
        temporal_point.y = vector[index].second;
        temporal_edges.points.push_back(temporal_point);
        temporal_point.x = vector[index+1].first;
        temporal_point.y = vector[index+1].second;
        temporal_edges.points.push_back(temporal_point);
      }
    }
  }
  //ROS_ASSERT(temporal_edges.points.size()%2 ==0);

  marker_array.markers.push_back(temporal_edges);
  map_publisher_.publish(marker_array);
}
