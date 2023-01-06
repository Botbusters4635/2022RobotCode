//
// Created by cc on 29/01/22.
//

#include "UseClimber.h"

UseClimber::UseClimber(const std::shared_ptr<PIDClimber> &climber, double setHeight, PIDClimber::FeedForward feedForward) {
    this->climber = climber;
    this->setHeight = setHeight;
    this->feedForward = feedForward;
}

void UseClimber::Initialize() {
    climber->setFF(feedForward);
}

void UseClimber::Execute() {
    climber->set(setHeight);
}

void UseClimber::End(bool interrupted) {
    ;
}

bool UseClimber::IsFinished() {
#ifdef SIMULATION
    return true;
#endif
    if (climber->isAtGoal()){return true;}else{return false;}
}