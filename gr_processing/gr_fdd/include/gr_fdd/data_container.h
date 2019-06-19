#ifndef DATA_CONTAINER_H
#define DATA_CONTAINER_H

#include <ros/ros.h>
#include <mutex>
#include <string>
//#include <boost/thread/thread.hpp>
//#include <boost/thread/mutex.hpp>
//boost::mutex mtx; 
#include <ctime>
 
namespace gr_fdd{
    typedef struct{
        const std::string NOTPUBLISH="TOPIC_NOT_PUBLISHED";
        const std::string SIGNAL_BIAS="BIAS_";//TODO Ignored from now, requires training or prepropcessing
        const std::string SIGNAL_DRIFT="DRIFT_";
        const std::string ACCURACY_LOSS="ACCURACY_LOSS_";
        const std::string SIGNAL_FREEZE="FREEZE_";
        const std::string DELAYED= "TOPIC_DELAY";
        //const std::string INVALID_VALUE = "Value is invalid"; // Maybe related to drifting
        //const std::string ANOMALY_DETECTED = "Anomaly on Data Detected"; // MAYBE RELATED TO ACCURACY LOSS?
        const std::string NOERROR = "No error detected";
    } ERRORID;

    class DataContainer{
        template<typename T>
        T variance(const std::list<T> &li, T mean)
        {
            size_t sz = li.size();
            if (sz == 1)
                return 0.0;
            // Calculate the mean
            //U mean_calc = std::accumulate(li.begin(), li.end(), 0.0) / sz;
            // Now calculate the variance
            auto variance_func = [&mean, &sz](T accumulator, const T& val)
            {
                return accumulator + ((val - mean)*(val - mean) / (sz - 1));
            };
            return std::accumulate(li.begin(), li.end(), 0.0, variance_func);
        }
 
    public:
        DataContainer(const std::string id, bool required_statistics=true, int samples_number=2, int window_size=10, double max_delay=0.1, double max_diff=0.1, double min_diff= 0.0);
        DataContainer(const DataContainer& other): samples_number_(std::move(other.samples_number_)),
                                                   data_id_(std::move(other.data_id_)), last_time_(ros::Time::now()),
                                                   container_status_(std::move(other.container_status_)),
                                                   errors_msgs_(std::move(other.errors_msgs_)),
                                                   window_size_(std::move(other.window_size_)), is_signal_delayed_(std::move(other.is_signal_delayed_)),
                                                   max_delay_(std::move(other.max_delay_)), max_diff_(other.max_diff_), min_diff_(other.min_diff_)
        {             
            
            //std::cout << data_id_ << samples_number_ << std::endl;
            //TODO
            this->data_.resize(samples_number_);
            this->window_mean_.resize(window_size_);
            this->window_std_.resize(window_size_);
            this->last_window_std_.resize(window_size_);
            //TODO
            this->check = std::bind(&DataContainer::statistics_check,this);

        }
        virtual ~DataContainer();
        bool default_check();
        bool statistics_check();
        std::function<bool(void)> check;
        void updateTime();
        std::string getId();
        void updateData(double new_data, int index=0);
        int getSamplesNumber();
        
        std::mutex mtx_;
        void reset();
        std::string getCurrentStatus();
    
    private:
        ros::Time last_time_;
        int window_size_;
        std::vector<double> window_mean_;
        std::vector<double> window_std_;
        std::vector<double> last_window_std_;
        double max_delay_;
        double max_diff_;
        double min_diff_;
        std::vector<std::list<double>> data_;
        std::string data_id_;
        bool is_signal_delayed_;
        int samples_number_;
        std::string container_status_;
        ERRORID errors_msgs_;
    };
};


#endif /* DATA_CONTAINER_H */

