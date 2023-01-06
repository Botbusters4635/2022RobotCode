//
// Created by cc on 04/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_CALCULATEHOODANGLE_H
#define BOTBUSTERS_REBIRTH_CALCULATEHOODANGLE_H
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <spdlog/spdlog.h>

#include "Math/InterpolatingTable/InterpolatingTable.h"

struct CalculateHoodAngleConfig{
    double minRange, maxRange;
    double minHoodVal, maxHoodVal;
};


class CalculateHoodAngle{
public:
    explicit CalculateHoodAngle(const CalculateHoodAngleConfig &config, const std::shared_ptr<InterpolatingTable> &interpolatingTable);

    double calculate(double targetDistance);

private:
    CalculateHoodAngleConfig config{};
    std::shared_ptr<spdlog::logger> log;
    std::shared_ptr<InterpolatingTable> interpolatingTable;
};


#endif //BOTBUSTERS_REBIRTH_CALCULATEHOODANGLE_H
