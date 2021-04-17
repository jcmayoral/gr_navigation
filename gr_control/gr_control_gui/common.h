#ifndef MYCOMMONVIZ_H
#define MYCOMMONVIZ_H

#include <vector>

#include <QWidget>
#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QPushButton>

#include <QGridLayout>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QLineEdit>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/marker_array_display.h>

#include <nodes_visualizer.hpp>

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

  class MyCommonViz: public QWidget{
    Q_OBJECT
    public:
      MyCommonViz( QWidget* parent = 0 );
      virtual ~MyCommonViz();

      void loadGUI();
      //private Q_SLOTS:
      
    protected:
      QGridLayout* controls_layout_;
      QVBoxLayout* main_layout_;
      rviz::RenderPanel* render_panel_;

    private:
      rviz::VisualizationManager* manager_;
  };
};
#endif 
