//
// Created by cc on 31/08/22.
//

#include "Shoot.h"

Shoot::Shoot(const std::shared_ptr<Feeder> &feeder){
    this->feeder = feeder;
    AddRequirements({feeder.get()});
}

void Shoot::Initialize() {
    hasRun = false;
    startTime = frc::Timer::GetFPGATimestamp().value();
}

void Shoot::Execute() {
    if (!hasRun){
        feeder->setFeederVol(-4);
        if (std::abs(frc::Timer::GetFPGATimestamp().value() - startTime) < 1){
            feeder->setFeederVol(12);
            hasRun = true;
        }
    }else{
        feeder->setFeederVol(12);
    }
}

void Shoot::End(bool interrupted) {

}

bool Shoot::IsFinished() {
    return false;
}