//
// Created by andrew on 03/11/21.
//

#include "InterpolatingTable.h"
#include <cmath>

InterpolatingTable::InterpolatingTable() = default;

void InterpolatingTable::addPoint(double x, double y) {
    table[x] = y;
    if(table.size() > 1) hasData = true;
}

void InterpolatingTable::addPointList(const std::vector<std::pair<double, double>> &list) {
    if (list.empty()){
        throw std::runtime_error("ShooterTableValues empty!");
    }
    for (const auto & i : list){
        double x = i.first;
        double y = i.second;
        addPoint(x, y);
    }
}

void InterpolatingTable::clearTable() {
    table.clear();
}

double InterpolatingTable::get(double x) {
    if(!hasData) return 0;
    if(table.empty()) return 0;

    auto it = table.find(x);
    if(it != table.end()){
        //Exact value found, return that
        return it->second;
    }

    //Return upper and lower bounds
    if(x <= table.begin()->first)
        return table.begin()->second;
    if(x >= table.rbegin()->first)
        return table.rbegin()->second;

    //Interpolate
    return interpolate(x);
}

double InterpolatingTable::interpolate(double x) {
    auto lb = --table.lower_bound(x);
    auto hb = table.upper_bound(x);

    if(lb == table.end()) throw std::runtime_error("lower bound");
    if(hb == table.end()) throw std::runtime_error("upper bound");

    const auto [x1,y1] = std::tie(lb->first, lb->second);
    const auto [x2,y2] = std::tie(hb->first, hb->second);

    return (y1 + ((x - x1) * ((y2 - y1) / (x2 - x1))));
}
