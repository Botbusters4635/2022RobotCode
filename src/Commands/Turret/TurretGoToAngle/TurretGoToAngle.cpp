//
// Created by cc on 11/06/22.
//

#include "TurretGoToAngle.h"

TurretGoToAngle::TurretGoToAngle(std::shared_ptr<PIDTurret> &turret, double radians) {
    this->turret = turret;
    this->radians = radians;
}

void TurretGoToAngle::Initialize() {
    ;
}

void TurretGoToAngle::Execute() {
    turret->set(radians);
}

void TurretGoToAngle::End(bool Interrupted) {
    ;
}
bool TurretGoToAngle::IsFinished() {
    return turret->atGoal();
}