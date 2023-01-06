//
// Created by abiel on 2/1/22.
//

#include "ClimberToLimitSwitch.h"

ClimberToLimitSwitch::ClimberToLimitSwitch(const std::shared_ptr<PIDClimber> &climber, double setpoint){
    this->climber = climber;
    this->setpoint = setpoint;
}

void ClimberToLimitSwitch::Initialize() {
    climber->usePIDControl(false);
}

void ClimberToLimitSwitch::Execute() {
    climber->setVoltage(setpoint);
}

void ClimberToLimitSwitch::End(bool interrupted) {
    climber->setVoltage(0);
    climber->resetToZero();
    climber->set(climber->getHeight() + 0.01_m);
    climber->usePIDControl(true);
}

bool ClimberToLimitSwitch::IsFinished() {
    return climber->getLimitSwitch();
}