//
// Created by cc on 19/05/22.
//

#include "SetTurret.h"

SetTurret::SetTurret(const std::shared_ptr<PIDTurret> &turret, double set) {
    this->turret = turret;
    this->set = set;
    AddRequirements(turret.get());
}

void SetTurret::Initialize() {
    ;
}

void SetTurret::Execute() {
    turret->set(set);
}
void SetTurret::End(bool interrupted) {
    ;
}
bool SetTurret::IsFinished() {
    return std::abs(turret->getHeading().value() - set) < 0.1;
}