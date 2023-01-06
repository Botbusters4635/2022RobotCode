//
// Created by cc on 10/02/22.
//

#include "SetHood.h"

SetHood::SetHood(const std::shared_ptr<Shooter> &shooter, double setState) {
    this->shooter = shooter;
    this->setState = setState;
    AddRequirements(shooter.get());
}

void SetHood::Initialize() {
    startTime = frc::Timer::GetFPGATimestamp().value();
}

void SetHood::Execute() {
    shooter->setHood(setState);
}

void SetHood::End(bool interrupted) {
    ;
}

bool SetHood::IsFinished() {
    if (std::abs(frc::Timer::GetFPGATimestamp().value() - startTime) > 0.1){
        return true;
    }else { return false; }
}
