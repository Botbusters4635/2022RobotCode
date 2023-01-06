//
// Created by cc on 23/11/21.
//

#include "IsOnTarget.h"
#include <frc/geometry/Pose2d.h>


IsOnTarget::IsOnTarget(const std::shared_ptr<EctoSwerve> &_swerve, double _tX, double _tY, double _tol) {
	tX = _tX;
	tY = _tY;
	tol = _tol;
	swerve = _swerve;
    AddRequirements(swerve.get());
}

void IsOnTarget::Initialize() { ; }

void IsOnTarget::Execute() { ; }

void IsOnTarget::End(bool interrupted) { ; }


bool IsOnTarget::IsFinished() {
	auto swervePose = swerve->getPose();
	double distance = std::hypot(tX - swervePose.X().value(), tY - swervePose.Y().value());
	return distance < tol;
}

