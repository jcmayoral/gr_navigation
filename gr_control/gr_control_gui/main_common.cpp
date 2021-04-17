#include <QApplication>
#include <common.h>
#include <ros/ros.h>

using namespace gr_control_gui;

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );
  MyCommonViz* myviz = new MyCommonViz();
  myviz->loadGUI();
  myviz->show();

  app.exec();

  delete myviz;
  return 1;
}
