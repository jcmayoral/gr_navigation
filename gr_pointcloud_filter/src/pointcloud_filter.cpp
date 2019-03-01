// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include <gr_pointcloud_filter/pointcloud_filter.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(gr_pointcloud_filter::MyNodeletClass, nodelet::Nodelet)

namespace gr_pointcloud_filter
{
    void MyNodeletClass::onInit()
    {

      for (auto i=0; i<100; ++i)
        std::cout << i << std::endl;
      NODELET_DEBUG("Initializing nodelet...");
    }
}
