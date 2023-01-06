//
// Created by abiel on 11/3/21.
//

#include "HolonomicPathFollower.h"
#include <frc/Timer.h>
#include <units/angular_acceleration.h>
#include <frc/trajectory/TrapezoidProfile.h>

HolonomicPathFollower::HolonomicPathFollower(const std::shared_ptr<EctoSwerve> &swerve, const frc::Trajectory &traj,
                                             const HolonomicPathFollowerConfig &config) {
	this->config = config;
	ff = config.ff;
	
	this->traj = traj;
	this->swerve = swerve;
	
	xController = std::make_unique<frc2::PIDController>(
			config.posConfig.p, config.posConfig.i, config.posConfig.d
	);
	
	yController = std::make_unique<frc2::PIDController>(
			config.posConfig.p, config.posConfig.i, config.posConfig.d
	);
	
	auto thetaControllerConstraints = frc::TrapezoidProfile<units::radians>::Constraints(
			units::radians_per_second_t(config.maxAngularVelocity),
			units::radians_per_second_squared_t(config.maxAngularAcceleration)
	);
	
	thetaController = std::make_unique<frc::ProfiledPIDController<units::radians>>(
			config.thetaConfig.p, config.thetaConfig.i, config.thetaConfig.d, thetaControllerConstraints
	);
	
	thetaController->EnableContinuousInput(units::radian_t(-M_PI), units::radian_t(M_PI));
}

void HolonomicPathFollower::Initialize() {
	table = nt::NetworkTableInstance::GetDefault().GetTable("HolonomicPathFollower");

	startTime = frc::Timer::GetFPGATimestamp().value();

    xController->Reset();
    yController->Reset();
	thetaController->Reset(swerve->getPose().Rotation().Radians());
}

void HolonomicPathFollower::Execute() {
	double time = getTime();
	auto state = traj.Sample(units::second_t(time));
	auto angle = state.pose.Rotation().Radians().value();
	double vel = state.velocity.value();
	double accel = state.acceleration.value();

    table->GetEntry("Velocity").SetDouble(vel);
    table->GetEntry("Accel").SetDouble(accel);

	if (time >= traj.TotalTime().value()) {
		vel = 0;
		accel = 0;
	}
	
	//Calculate ff vector (units are in volts)
	auto ffVec = ff.calculateFF(fromAngle(angle, vel), fromAngle(angle, accel));
	
	auto swervePose = swerve->getPose();
	auto rotSet = config.rotationFunction(swervePose.X().value(), swervePose.Y().value()).Radians();
	auto xOut = xController->Calculate(swervePose.X().value(), state.pose.X().value());
	auto yOut = yController->Calculate(swervePose.Y().value(), state.pose.Y().value());
	auto rotOut = thetaController->Calculate(swervePose.Rotation().Radians(), rotSet);
    double distanceToTarget = std::hypot(swervePose.X().value() - state.pose.X().value(), swervePose.Y().value() - state.pose.Y().value());
    table->GetEntry("DistanceToSetpoint").SetDouble(distanceToTarget);
	frc::ChassisSpeeds targetVel;
	//Add FF to target voltage
	targetVel.vx = units::meters_per_second_t(xOut + ffVec.x);
	targetVel.vy = units::meters_per_second_t(yOut + ffVec.y);
	targetVel.omega = units::radians_per_second_t(rotOut);
	targetVel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(targetVel.vx, targetVel.vy, targetVel.omega,
	                                                        swervePose.Rotation());

    //targetVel.vx += units::meters_per_second_t(ffVec.x);
    //targetVel.vy += units::meters_per_second_t(ffVec.y);

	swerve->setVoltage(targetVel);
	updateTelemetry(state, ffVec, targetVel);
	//frc::Field2d fieldTest;
//    frc::FieldObject2d robotObj;
//    robotObj.SetPose(units::meter_t(swervePose.X()), units::meter_t(swervePose.Y()), swervePose.Rotation().Radians());
//    fieldTest.GetObject("robotObj");
	
	//fieldTest.SetRobotPose(units::meter_t(swervePose.X()), units::meter_t(swervePose.Y()), swervePose.Rotation().Radians());
	//frc::SmartDashboard::PutData("field", &fieldTest);
//    frc::SmartDashboard::Delete("field");

}

void HolonomicPathFollower::End(bool interrupted) {
	if (config.stopWhenFinished || interrupted) {
		swerve->setVelocity(frc::ChassisSpeeds());
	}
}

bool HolonomicPathFollower::IsFinished() {
	if (config.runUntilPathFinished) {
		auto endPose = traj.Sample(traj.TotalTime()).pose;
		frc::Pose2d swervePose = swerve->getPose();
		auto dist = std::hypot((endPose.X() - swervePose.X()).value(), (endPose.Y() - swervePose.Y()).value());
		return dist < config.endTolerance;
	} else {
		return getTime() >= traj.TotalTime().value();
	}
	
	return true;
}

void HolonomicPathFollower::updateTelemetry(const frc::Trajectory::State &state, const frc::Vector2d &ffVec,
                                            const frc::ChassisSpeeds &out) {
	ffXOutEntry.SetDouble(ffVec.x);
	ffYOutEntry.SetDouble(ffVec.y);
	
	xOutEntry.SetDouble(out.vx.value());
	yOutEntry.SetDouble(out.vy.value());
	thetaOutEntry.SetDouble(out.omega.value());
	
	velEntry.SetDouble(state.velocity.value());
	accelEntry.SetDouble(state.acceleration.value());
	
	pathStateXEntry.SetDouble(state.pose.X().value());
	pathStateYEntry.SetDouble(state.pose.Y().value());
}
