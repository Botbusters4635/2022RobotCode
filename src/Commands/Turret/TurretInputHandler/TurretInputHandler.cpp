//
// Created by cc on 05/09/22.
//

#include "TurretInputHandler.h"

bool TurretInputHandler::registeredJoysticks = false;

TurretInputHandler::TurretInputHandler(const std::shared_ptr<PIDTurret> &turret) {
    this->turret = turret;
    AddRequirements({this->turret.get()});
}

void TurretInputHandler::Initialize() {
    if (!registeredJoysticks){
        input.registerAxis(turretAxis, "leftX", 1);
        input.registerButton(lock, "Y", 1);
        input.registerButton(manualTurret, "leftBumper", 1);
        input.registerButton(endCommand, "X", 1);
        registeredJoysticks = true;
    }
}

void TurretInputHandler::Execute() {
        turret->usePIDControl(false);
        turret->useSoftLimits(false);
        turret->setVoltage((-turretAxis.get() * 12));

        if (lock.get()){
            turret->setVoltage(0);
            turret->setSensorAngle(250_deg);
            turret->resetController();
        }

}

void TurretInputHandler::End(bool interrupted) {
    turret->setVoltage(0);
    turret->useSoftLimits(true);
    turret->usePIDControl(true);
}

bool TurretInputHandler::IsFinished() {
#ifdef SIMULATION
    return false;
#endif
    return endCommand.get();
}