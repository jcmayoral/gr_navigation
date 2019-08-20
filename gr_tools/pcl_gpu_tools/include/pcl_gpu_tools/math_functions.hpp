template<typename T>
        double getAbsoluteRange(std::vector<T> v){
            auto min_value = *std::min_element(std::begin(v), std::end(v));
            auto max_value = *std::max_element(std::begin(v), std::end(v));
            return std::fabs(max_value-min_value);
            }

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
template<typename T>
        double calculateVariance(std::vector<T> v){
            auto sum = std::accumulate(std::begin(v), std::end(v), 0.0);
            auto m =  sum / v.size();
            auto accum = 0.0;

            std::for_each (std::begin(v), std::end(v), [&](const double d) {
                accum += (d - m) * (d - m);
            });

            return accum / (v.size()-1);
            }

template<typename T>
    double calculateStd(std::vector<T> v){
        auto sum = std::accumulate(std::begin(v), std::end(v), 0.0);
        auto m =  sum / v.size();
        auto accum = 0.0;

        std::for_each (std::begin(v), std::end(v), [&](const double d) {
            accum += (d - m) * (d - m);
        });

        return sqrt(accum / (v.size()-1));
    }
