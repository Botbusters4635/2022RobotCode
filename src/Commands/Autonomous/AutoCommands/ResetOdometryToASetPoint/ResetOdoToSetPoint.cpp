//
// Created by cc on 13/01/22.
//

#include "ResetOdoToSetPoint.h"

ResetOdoToSetPoint::ResetOdoToSetPoint(const std::shared_ptr<EctoSwerve> &swerve, frc::Pose2d pose2d) {
    this->swerve = swerve;
    this->pose2d = pose2d;
    AddRequirements(swerve.get());
}

void ResetOdoToSetPoint::Initialize() {
    ;
}

void ResetOdoToSetPoint::Execute() {
    swerve->resetOdometry(pose2d);
}

void ResetOdoToSetPoint::End(bool interrupted) {
    ;
}

bool ResetOdoToSetPoint::IsFinished() {
    return true;
}