#include <common.h>

using namespace gr_control_gui;

MyCommonViz::MyCommonViz( QWidget* parent): QWidget( parent ){
  main_layout_ = new QVBoxLayout();
  render_panel_ = new rviz::RenderPanel();
  controls_layout_ = new QGridLayout();
  QPushButton* load_topological_map = new QPushButton ("Load Map");
  controls_layout_->addWidget( load_topological_map, 0, 0 );
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  std::cout << "C" << std::endl;
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
  proximity_marker = manager_->createDisplay( "rviz/MarkerArray", "proximity_marker", true );
  ROS_ASSERT( proximity_marker != NULL );
  proximity_marker->subProp( "Marker Topic" )->setValue("proximity_visualization");

  rviz::Display* robot_display;
  robot_display = manager_->createDisplay( "rviz/RobotModel", "thorvald", true );
  ROS_ASSERT( robot_display != NULL );
  //robot_display->subProp( "Marker Topic" )->setValue("region");

  rviz::Display* robot_path;
  robot_path = manager_->createDisplay( "rviz/Path", "robot_path", true );
  ROS_ASSERT( robot_path != NULL );
  robot_path->subProp( "Topic" )->setValue("gr_sbpl_trajectory_generator_node/plan");
  //robot_path->subProp( "Pose Style" )->setValue(2);
  
  rviz::Display* map;
  map = manager_->createDisplay( "rviz/Map", "safety_map", true );
  ROS_ASSERT( map != NULL );
  map->subProp( "Topic" )->setValue("/grid_map_visualization/safety_costmap");

  rviz::Display* pc;
  pc = manager_->createDisplay( "rviz/PointCloud2", "velodyne_pc", true );
  ROS_ASSERT( pc != NULL );
  pc->subProp( "Topic" )->setValue("/velodyne_points");


  rviz::Display* dect;
  dect = manager_->createDisplay( "jsk_rviz_plugin/BoundingBoxArray", "dect_bb", true );
  ROS_ASSERT( dect != NULL );
  dect->subProp( "Topic" )->setValue("/detection/bounding_boxes");  

  manager_->setFixedFrame("map");
  manager_->initialize();
  manager_->startUpdate();
}

void MyCommonViz::loadGUI(){
  main_layout_->addLayout( controls_layout_ );
  main_layout_->addWidget( render_panel_ );
  setLayout( main_layout_ );
}

// Destructor.
MyCommonViz::~MyCommonViz(){
  delete manager_;
}
