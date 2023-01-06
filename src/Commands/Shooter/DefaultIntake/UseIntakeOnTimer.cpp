//
// Created by cc on 05/02/22.
//

#include "UseIntakeOnTimer.h"

UseIntakeOnTimer::UseIntakeOnTimer(const std::shared_ptr<Intake> &intake, double target, bool state, double time) {
    this->intake = intake;
    this->time = time;
    this->state = state;
    this->target = target;
    AddRequirements(intake.get());
}

void UseIntakeOnTimer::Initialize() {
    startTime = frc::Timer::GetFPGATimestamp().value();
}

void UseIntakeOnTimer::Execute() {
    if ((frc::Timer::GetFPGATimestamp().value() - startTime) < time) {
        intake->set(target, state);
    }else {
        intake->set(0, false);
    }
}

void UseIntakeOnTimer::End(bool interrupted) {
    ;
}

bool UseIntakeOnTimer::IsFinished() {
    if ((frc::Timer::GetFPGATimestamp().value() - startTime) < time + 0.01) {
        return false;
    }else{
        return true;
    }

}


