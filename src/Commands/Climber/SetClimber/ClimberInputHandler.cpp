//
// Created by cc on 05/09/22.
//

#include "ClimberInputHandler.h"

ClimberInputHandler::ClimberInputHandler(const std::shared_ptr<PIDClimber> &climber) {
    this->climber = climber;
    AddRequirements({climber.get()});

}

void ClimberInputHandler::Initialize() {
    climber->usePIDControl(false);
    climber->useSoftLimits(true);
    if (!registeredJoysticks){
        input.registerAxis(climberAxis, "rightY", 1);
        input.registerButton(releaseClimber, "A", 1);
        registeredJoysticks = true;
    }
}

void ClimberInputHandler::Execute() {
    climber->setVoltage((-climberAxis.get() * 12));
    if (releaseClimber.get()){
//        climber->releaseClimber();
    }
}

void ClimberInputHandler::End(bool interrupted) {
    climber->setVoltage(0);
}

bool ClimberInputHandler::IsFinished() {
#ifdef SIMULATION
    return false;
#endif
    return false;
}