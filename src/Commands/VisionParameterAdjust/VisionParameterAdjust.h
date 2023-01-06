//
// Created by abiel on 1/26/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONPARAMETERADJUST_H
#define BOTBUSTERS_REBIRTH_VISIONPARAMETERADJUST_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Core/VisionManager/VisionManager.h"
#include "Math/InterpolatingTable/InterpolatingTable.h"
#include <functional>

enum class VisionAdjustType {
    Distance, //Returns distance to target in meters
    YawError //Returns 0 when aligned to target
};

struct VisionAdjustParam {
    /**
     * Represents an adjustable parameter based on a vision measurement
     * @param table Interpolating table
     * @param func Output function (y)
     * @param atSetpoint Optional, if given, command waits until all setpoints are reached to finish
     */
    VisionAdjustParam(const InterpolatingTable &table,
                      const std::function<void(double y)> &func,
                      const std::function<bool()> &atSetpoint,
                      VisionAdjustType type = VisionAdjustType::Distance){
        this->table = table;
        if(!table.ready())
            throw std::runtime_error("Invalid interpolating table!");
        this->func = func;
        this->atSetpoint = atSetpoint;
        this->type = type;
        setpointFuncGiven = true;
        hasInterpTable = true;
    }

    VisionAdjustParam(const InterpolatingTable &table,
                      const std::function<void(double y)> &func,
                      VisionAdjustType type = VisionAdjustType::Distance) :
            VisionAdjustParam(table, func, []{return true;}, type) {
        setpointFuncGiven = false;
    }

    VisionAdjustParam(const std::function<void(double y)> &func,
                      const std::function<bool()> &atSetpoint,
                      VisionAdjustType type = VisionAdjustType::YawError){
        this->func = func;
        this->atSetpoint = atSetpoint;
        this->type = type;
        setpointFuncGiven = true;
        hasInterpTable = false;
    }

    VisionAdjustParam(const std::function<void(double y)> &func,
                      VisionAdjustType type = VisionAdjustType::YawError) :
                      VisionAdjustParam(func, []{return true;}, type){
        setpointFuncGiven = false;
    }

    InterpolatingTable table;
    bool hasInterpTable = false;
    std::function<void(double y)> func;
    std::function<bool()> atSetpoint;
    bool setpointFuncGiven = false;
    VisionAdjustType type;
};

//TODO Better name?
/**
 * Adjusts _any_ parameter based on a (optional) given linear interpolating table and functor
 * Note!! Only finishes when a valid target is found, and all given params have reached their setpoints
 */
class VisionParameterAdjust : private Module, public frc2::CommandHelper<frc2::CommandBase, VisionParameterAdjust> {
public:
    VisionParameterAdjust(const std::shared_ptr<VisionManager> &manager, std::vector<VisionAdjustParam> &&params, const std::vector<frc2::Subsystem*> &requirements);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<VisionManager> manager;
    std::vector<VisionAdjustParam> params;
};


#endif //BOTBUSTERS_REBIRTH_VISIONPARAMETERADJUST_H
