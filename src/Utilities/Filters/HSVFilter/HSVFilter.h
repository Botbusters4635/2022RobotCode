//
// Created by cc on 06/08/22.
//

#ifndef BOTBUSTERS_REBIRTH_HSVFILTER_H
#define BOTBUSTERS_REBIRTH_HSVFILTER_H

#include <bits/stdc++.h>
#include <frc/smartdashboard/SmartDashboard.h>

struct HSVFilterConfig{
    std::vector<double> low = {0,0,0};
    std::vector<double> high = {0,0,0};
};

class HSVFilter {
public:
    explicit HSVFilter(const HSVFilterConfig &config);

    void setThresholds(std::vector<double> low, std::vector<double> high);


    //calculate uses 0-100 s and v values, 0-360 h values.
    bool calculate(double h, double s, double v);

    bool calculate(std::vector<double> hsv){
        return calculate(hsv[0], hsv[1], hsv[2]);
    }

private:
    std::vector<double> low, high;
};


#endif //BOTBUSTERS_REBIRTH_HSVFILTER_H
