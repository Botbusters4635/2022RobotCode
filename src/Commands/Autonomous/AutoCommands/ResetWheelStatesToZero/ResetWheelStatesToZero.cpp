//
// Created by cc on 06/01/22.
//

#include "ResetWheelStatesToZero.h"

ResetWheelStateToZero::ResetWheelStateToZero(const std::shared_ptr<EctoSwerve> &swerve, frc::Pose2d pose2d) {
	this->swerve = swerve;
    this->pose2d = pose2d;
    AddRequirements(swerve.get());
}

void ResetWheelStateToZero::Initialize() {
	;
}

void ResetWheelStateToZero::Execute() {
	std::array<frc::SwerveModuleState, 4> states;
	swerve->setModules(states);
	swerve->resetOdometry(pose2d);
}

void ResetWheelStateToZero::End(bool interrupted) {
	;
}

bool ResetWheelStateToZero::IsFinished() {
	return true;
}