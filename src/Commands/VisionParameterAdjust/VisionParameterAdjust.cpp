//
// Created by abiel on 1/26/22.
//

#include "VisionParameterAdjust.h"

VisionParameterAdjust::VisionParameterAdjust(const std::shared_ptr<VisionManager> &manager, std::vector<VisionAdjustParam> &&params, const std::vector<frc2::Subsystem*> &requirements):
    Module("VisionParameterAdjust"){
    this->manager = manager;
    this->params = params;
    AddRequirements(requirements);
}

void VisionParameterAdjust::Initialize() {
    if(std::all_of(params.begin(), params.end(), [](const auto &param){
        return !param.setpointFuncGiven;
    })){
        log->warn("Command will finish as soon as a target was found, as no setpoint funcs were given!");
    }
}

void VisionParameterAdjust::Execute() {
    auto yaw = manager->getYawError();
    auto distance = manager->getTargetDistance();
    for(auto &param : params){
        double x;
        if(param.type == VisionAdjustType::Distance){
            x = distance;
        } else{
            x = yaw;
        }

        double y;
        if(param.hasInterpTable)
            y = param.table.get(x);
        else
            y = x; //1:1 mapping
        param.func(y);
    }
}

/**
 * Only finish when target is found and all params are at setpoint
 * @return
 */
bool VisionParameterAdjust::IsFinished() {
    return false;
    return manager->hasValidTarget() &&
        std::all_of(params.begin(), params.end(), [](auto &param){
            return param.atSetpoint();
        });
}

void VisionParameterAdjust::End(bool interrupted) {
    ;
}