//
// Created by cc on 25/01/22.
//

#include "FeedShooter.h"

FeedShooter::FeedShooter(const std::shared_ptr<Feeder> &feeder, double targetVoltage, bool endInstanly) {
    this->feeder = feeder;
    this->targetVoltage = targetVoltage;
    this->endInstantly =  endInstanly;
    AddRequirements(feeder.get());
}

void FeedShooter::Initialize() {
    ;
}

void FeedShooter::Execute() {
    feeder->setFeederVol(targetVoltage);
}

void FeedShooter::End(bool interrupted) {
//    feeder->setFeederVol(0);
}

bool FeedShooter::IsFinished() {
    return endInstantly;
}