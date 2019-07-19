#include <sensor_msgs/PointCloud2.h>
#include <gr_pointcloud_filter/FiltersConfig.h>

namespace gr_pointcloud_filter{
    class CustomCUDAManager{
        public:
            CustomCUDAManager();
            ~CustomCUDAManager();
            void reconfigure(gr_pointcloud_filter::FiltersConfig &config);
		    void test(sensor_msgs::PointCloud2 pc);
            std::string getID(){
	        	return id_.c_str();
            }
        private:
            std::string id_;
    };
}