#include <ros/ros.h>
#include <gr_fdd/monitor.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "magazino_fault_detection");
  MainMonitor main_monitor;
}
