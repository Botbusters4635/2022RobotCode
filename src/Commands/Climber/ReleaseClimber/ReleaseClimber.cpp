//
// Created by cc on 29/01/22.
//

#include "ReleaseClimber.h"

ReleaseClimber::ReleaseClimber(const std::shared_ptr<PIDClimber> &climber, bool state) {
    this->climber = climber;
    this->state = state;
}

void ReleaseClimber::Initialize() {
    startTime = frc::Timer::GetFPGATimestamp().value();
}

void ReleaseClimber::Execute() {
//    climber->releaseClimber();

}

void ReleaseClimber::End(bool interrupted) {
    ;
}

bool ReleaseClimber::IsFinished() {
    if (std::abs(frc::Timer::GetFPGATimestamp().value() - startTime) > 0.2){
        return true;
    }else { return false; }
}