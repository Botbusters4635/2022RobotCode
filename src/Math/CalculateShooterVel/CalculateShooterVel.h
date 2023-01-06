//
// Created by cc on 05/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_CALCULATEHOODVELOCITY_H
#define BOTBUSTERS_REBIRTH_CALCULATEHOODVELOCITY_H

#include <memory>
#include "Math/InterpolatingTable/InterpolatingTable.h"

struct CalculateShooterVelConfig{
    double minRange, maxRange;
    double minVel, maxVel;
};


class CalculateShooterVel {
public:
    explicit CalculateShooterVel(const CalculateShooterVelConfig &config, const std::shared_ptr<InterpolatingTable> &interpolatingTable);

    double calculate(double targetDistance);

private:
    CalculateShooterVelConfig config;

    std::shared_ptr<InterpolatingTable> interpolatingTable;

};


#endif //BOTBUSTERS_REBIRTH_CALCULATEHOODVELOCITY_H
