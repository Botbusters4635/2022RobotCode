//
// Created by andrew on 03/11/21.
//

#ifndef BOTBUSTERS_REBIRTH_INTERPOLATINGTABLE_H
#define BOTBUSTERS_REBIRTH_INTERPOLATINGTABLE_H

#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <optional>

class InterpolatingTable {
public:
	explicit InterpolatingTable();
	
	void addPoint(double x, double y);

    void addPointList(const std::vector<std::pair<double, double>> &list);

//    void removePoint(const std::vector<std::pair<double, double>> &list);
	
	double get(double x);

    bool ready() const {
        return hasData;
    }

    void clearTable();

private:
	double interpolate(double x);
	
	std::map<double, double> table;
	
	bool hasData = false;
	
};


#endif //BOTBUSTERS_REBIRTH_INTERPOLATINGTABLE_H
