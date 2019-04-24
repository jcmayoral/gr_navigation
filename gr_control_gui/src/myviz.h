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
#include <QSpinBox>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/marker_array_display.h>

#include "nodes_visualizer.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <mongodb_store/message_store.h>

#include <boost/foreach.hpp>


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
      void setTerrainWidth( int value);
      void setTerrainHeight( int value);
      void setRobotRadius(int radius);

      void executeTopoMap();
      void visualizeMap();
      void saveMap();
      void deleteTopoMap(std::string map_id);

    private:
      rviz::VisualizationManager* manager_;
      rviz::RenderPanel* render_panel_;
      rviz::Display* marker_display_;
      MapGenerator* map_utils_;
      int height_cells_;
      int width_cells_;
      ros::NodeHandle nh_;
      ros::Publisher map_publisher_;
      float robot_radius_;
      float terrain_width_;
      float terrain_height_;
      visualization_msgs::MarkerArray marker_array_;
     	mongodb_store::MessageStoreProxy* message_store_;

      NodeMap node_map_;
      std::vector<Edges> edges_;
  };
};
  // END_TUTORIAL
#endif // MYVIZ_H
