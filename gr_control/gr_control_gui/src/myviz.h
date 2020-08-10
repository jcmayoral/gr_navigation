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
#ifndef MYVIZ_H
#define MYVIZ_H

#include <vector>

#include <QWidget>
#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QPushButton>

#include <QGridLayout>
#include <QVBoxLayout>
#include <QDoubleSpinBox>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/marker_array_display.h>

#include "nodes_visualizer.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <navigation_msgs/TopologicalMap.h>
#include <mongodb_store/message_store.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <boost/foreach.hpp>
#include <thread>

#include <gr_map_utils/UpdateMap.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

namespace gr_control_gui{
  typedef std::map<std::string, geometry_msgs::Pose> NodeMap;
  typedef std::pair<std::string, std::string> Edges;

  class MyViz: public QWidget{
    Q_OBJECT
    public:
      MyViz( QWidget* parent = 0 );
      virtual ~MyViz();

      private Q_SLOTS:
      void setTerrainY( int value);
      void setTerrainX( int value);
      void setDesiredRow(int row);
      void executeTopoMap();
      void visualizeMap();
      void saveMap();
      void deleteTopoMap();

      void publishRegion();
      void timetogoCB(const std_msgs::Float32ConstPtr time2go);
      void executeCycle(int cycle);
    private:
      rviz::VisualizationManager* manager_;
      rviz::RenderPanel* render_panel_;
      MapGenerator* map_utils_;
      int x_cells_;
      int y_cells_;
      int current_row_;
      ros::NodeHandle nh_;
      ros::Publisher map_publisher_;
      ros::Publisher reset_publisher_;
      ros::Publisher region_publisher_;
      ros::ServiceClient update_client_;

      double robot_radius_;
      float terrain_y_;
      float terrain_x_;
      visualization_msgs::MarkerArray marker_array_;
     	mongodb_store::MessageStoreProxy* message_store_;

      NodeMap node_map_;
      std::vector<Edges> edges_;

      std::string storing_id_;

      QLabel* time_to_go;
      ros::Subscriber time_to_go_sub_;
      int max_numberrows_;

      std::thread* t1;
  };
};
  // END_TUTORIAL
#endif // MYVIZ_H
