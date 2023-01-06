//
// Created by cc on 05/02/22.
//

#include "SpinupShooter.h"

SpinupShooter::SpinupShooter(const std::shared_ptr<Shooter> &shooter, double defaultValue) {

    this->shooter = shooter;
    this->defaultValue = defaultValue;
    AddRequirements(shooter.get());
}

void SpinupShooter::Initialize() {
    ;
}

void SpinupShooter::Execute() {
    shooter->setSetpoint(defaultValue);
}
void SpinupShooter::End(bool interrupted) {
    ;
}
bool SpinupShooter::IsFinished() {
    return std::abs(shooter->getVelocity() - defaultValue) < 50;
}

