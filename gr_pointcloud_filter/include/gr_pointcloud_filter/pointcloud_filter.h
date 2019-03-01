#include <nodelet/nodelet.h>

namespace gr_pointcloud_filter
{

    class MyNodeletClass : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
    };

}
