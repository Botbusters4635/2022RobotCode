//
// Created by cc on 08/10/22.
//

#include "WaitTurretOnTarget.h"
#include <frc/smartdashboard/SmartDashboard.h>

WaitTurretOnTarget::WaitTurretOnTarget(const std::shared_ptr<PIDTurret> &turret) {
    this->turret = turret;
}

void WaitTurretOnTarget::Initialize() {
    ;
}

void WaitTurretOnTarget::Execute() {
    ;
}
void WaitTurretOnTarget::End(bool interrupted) {
    ;
}
bool WaitTurretOnTarget::IsFinished() {
    return turret->atGoal();
}