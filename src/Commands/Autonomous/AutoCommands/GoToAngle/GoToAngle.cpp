//
// Created by cc on 06/01/22.
//

#include "GoToAngle.h"

GoToAngle::GoToAngle(const std::shared_ptr<EctoSwerve> &swerve, double angle, double tol) {
	this->swerve = swerve;
	this->angle = angle * (M_PI / 180.0);
	this->tol = tol * (M_PI / 180.0);
    AddRequirements(swerve.get());
}

void GoToAngle::Initialize() {
	anglePID.EnableContinuousInput(units::radian_t(-M_PI), units::radian_t (M_PI));
    anglePID.SetGoal(units::radian_t(angle));
    anglePID.SetTolerance(units::radian_t(tol), units::radians_per_second_t(31));
    anglePID.Reset(swerve->getPose().Rotation().Radians());

}

void GoToAngle::Execute() {
	state = swerve->getYaw();
	pidOut = anglePID.Calculate(units::radian_t(state));
	chassisSpeeds.omega = units::radians_per_second_t(pidOut);
	swerve->setPercent(chassisSpeeds);
}

void GoToAngle::End(bool interrupted) {
	chassisSpeeds.omega = units::radians_per_second_t(0);
	swerve->setPercent(chassisSpeeds);
}

bool GoToAngle::IsFinished() {
	return anglePID.AtGoal();
}