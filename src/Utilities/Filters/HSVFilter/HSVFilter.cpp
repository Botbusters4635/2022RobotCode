//
// Created by cc on 06/08/22.
//

#include "HSVFilter.h"


HSVFilter::HSVFilter(const HSVFilterConfig &config) {
    if (config.low.empty() || config.high.empty()){
        throw std::runtime_error("empty thresholds given to HSVFilter (Initialize)");
    }
    this->low = config.low ;
    this->high  = config.high;
}

void HSVFilter::setThresholds(std::vector<double> low, std::vector<double> high) {
    if (low.empty() || high.empty()){
        throw std::runtime_error("empty thresholds given to HSVFilter (setThresholds)");
    }
    this->low = low;
    this->high  = high;
}


//true if inside range
bool HSVFilter::calculate(double h, double s, double v) {
    if (low.empty() || high.empty()){
        throw std::runtime_error("low or high thresholds are emtpy");
    }
    if (h >= low[0] && h < high[0]){
        if (s >= low[1] && s < high[1]){
            if (v >= low[2] && v < high[2]){
                return true;
            }
        }
    }

    return false;



}