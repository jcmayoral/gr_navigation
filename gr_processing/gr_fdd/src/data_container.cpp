#include <gr_fdd/data_container.h>
#include <boost/thread/pthread/recursive_mutex.hpp>

using namespace gr_fdd;

int DataContainer::getSamplesNumber(){
    return samples_number_;
}


void DataContainer::updateTime(){
    last_time_ = ros::Time::now();
}

void DataContainer::updateData(double new_data, int index){
    //std::cout << index <<  "," << data_.size() << std::endl;
    data_[index].push_front(new_data);
    
    if (data_[index].size() > window_size_){
        data_[index].pop_back();
    }
}

void DataContainer::reset(){
    for (int i=0; i<samples_number_;++i)
        data_[i].clear();
}

DataContainer::DataContainer(const std::string id, bool required_statistics, int samples_number, int window_size, double max_delay, double max_diff, double min_diff): last_time_(ros::Time::now()),
        window_size_(window_size), max_delay_(max_delay), data_id_(std::string(id)),
        is_signal_delayed_(false), samples_number_(samples_number), max_diff_(max_diff),
        min_diff_(min_diff)
        //container_status_(errors_msgs_.NOERROR)
{
    container_status_ = errors_msgs_.NOERROR;
    //std::strcpy(data_id_, id);
    if (required_statistics){
        check = std::bind(&DataContainer::statistics_check,this);
    }
    else{
        check = std::bind(&DataContainer::default_check,this);
    }
    
    //TODO
    data_.resize(samples_number);
    window_mean_.resize(samples_number);
    window_std_.resize(samples_number);
    last_window_std_.resize(samples_number);
}

std::string DataContainer::getId(){
    return data_id_;
}

std::string DataContainer::getCurrentStatus(){
    return container_status_;
}

bool DataContainer::default_check(){
    ROS_ERROR("THIS SHOULD NOT BE PRINTED");
    return false;
}

bool DataContainer::statistics_check(){
    std::lock_guard<std::mutex> lk(mtx_);
    bool result = false;
    container_status_ = "";//errors_msgs_.NOERROR;
    if (data_[0].size() < 2){
        ROS_INFO_ONCE("Waiting for data");
        container_status_ = errors_msgs_.NOTPUBLISH;
        return false;
    } 

    std::vector<std::list<double>>::iterator it = data_.begin();
    double new_mean; 
    double last_value;
    std::list<double> inner_list;
    bool freeze_bool = false;
    bool drift_flag = false;

    for (; it!= data_.end(); ++it){ //A ros message can contain several measurements
        new_mean = std::accumulate(it->begin(), it->end(), 0.0)/it->size();
        window_mean_[std::distance(data_.begin(), it)] = new_mean;
        window_std_[std::distance(data_.begin(), it)] = std::sqrt(variance(*it, new_mean));
        inner_list = *it;
        if (inner_list.size() < 1)
            break;

        last_value = inner_list.front();

        for (std::list<double>::iterator inner_it = inner_list.begin();inner_it!=inner_list.end(); ++inner_it){
            auto value_diff = fabs(last_value - *inner_it);

            if (value_diff < min_diff_){
                ROS_INFO_STREAM_THROTTLE(2, "Differential value " << value_diff);
                freeze_bool = true;
                //container_status_ += errors_msgs_.SIGNAL_FREEZE;
                result = true;
            }

            if (value_diff*inner_list.size() > max_diff_){
                ROS_INFO_STREAM_THROTTLE(2, "Maximum difference between values  "<< max_diff_ << " with value " << value_diff*inner_list.size());
                result = true;
                drift_flag = true;
                //container_status_ += errors_msgs_.SIGNAL_DRIFT;
            }
            last_value = *inner_it;
        }


    /*
     * For now avoiding a signal per measurment
        if (freeze_bool)
            container_status_ += errors_msgs_.SIGNAL_FREEZE;
        if (drift_flag)
            container_status_ += errors_msgs_.SIGNAL_DRIFT;
     */
    }

    if (freeze_bool)
        container_status_ += errors_msgs_.SIGNAL_FREEZE;
    if (drift_flag)
        container_status_ += errors_msgs_.SIGNAL_DRIFT;



    double delay = double((ros::Time::now()-last_time_).toSec());
    is_signal_delayed_ = false;
    //std::cout << "TIMER" << delay << "," << data_id_ <<std::endl;

    
    if (delay > max_delay_){
        is_signal_delayed_ = true;
        //ROS_WARN_STREAM("Signal delayed in "<< data_id_ << " by " <<delay << " seconds");
        container_status_ += errors_msgs_.DELAYED;
        result = true;
        //return true;
    }
    
    //std::cout << "DIFF on " << data_id_ << " is "<< delay_ << std::endl;
    for (int i=0; i< samples_number_; ++i){
        if (fabs(window_std_[i] - last_window_std_[i])> 0.4){
            //ROS_WARN_STREAM("Anomaly detected on  index " << i);
            //q:q>[q
            ROS_WARN_STREAM("Anomaly detected on  " << data_id_);// << " with rate " << fabs(window_std_/last_window_std_));
            container_status_ += errors_msgs_.ACCURACY_LOSS;
            result = true;
        }
        last_window_std_[i] = window_std_[i];
    }
    
    if (container_status_.empty())
        container_status_ = errors_msgs_.NOERROR;

    //lock.unlock();
    //mtx->unlock();
    //ROS_INFO_THROTTLE(5, "ALL GOOD");
    //std::cout << container_status_ << std::endl;
    last_window_std_ = window_std_;
    return result;
}

DataContainer::~DataContainer() {
}

