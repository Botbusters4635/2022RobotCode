//
// Created by abiel on 11/3/21.
//

#include "SwerveCharacterizationRobot.h"
#include <frc/kinematics/DifferentialDriveKinematics.h>

SwerveCharacterizationRobot::SwerveCharacterizationRobot(bool doRotation) : EctoCharacterizationRobot("SwerveCharacterizationRobot") {
	this->doRotation = doRotation;
}

void SwerveCharacterizationRobot::robotInit() {
	EctoSwerveConfig swerveConfig;
	swerveConfig.length = 0.5334;
	swerveConfig.width = 0.5334;
	swerveConfig.wheelCircumference = 0.0508 * 2 * M_PI;
	swerveConfig.gearRatio = 8.95;
	swerve = std::make_shared<EctoSwerve>(swerveConfig);
	
	auto &input = InputManager::getInstance();
	
    commandScheduler.RegisterSubsystem(swerve.get());
	input.registerAxis(leftSide, "leftY");
	input.registerAxis(rightSide, "rightY");
}

void SwerveCharacterizationRobot::teleopUpdate() {
	swerve->setModules(tankSwerve(leftSide.get() * 12.0, rightSide.get() * 12.0), MotorControlMode::Voltage);
}

void SwerveCharacterizationRobot::autoInit() {
    logger.UpdateThreadPriority();
    logger.InitLogging();
}

void SwerveCharacterizationRobot::autoUpdate() {
	auto state = swerve->getMotorStates();
	
	auto leftPos = (state.topLeft.wheelPosition + state.backLeft.wheelPosition) / 2.0;
	auto rightPos = (state.topRight.wheelPosition + state.backRight.wheelPosition) / 2.0;
	
	auto leftVel = (state.topLeft.wheelVelocity + state.backLeft.wheelVelocity) / 2.0;
	auto rightVel = (state.topRight.wheelVelocity + state.backRight.wheelVelocity) / 2.0;
	
	auto yaw = swerve->getRawYaw();
	auto angularRate = swerve->getYawRate();

    double left = logger.GetLeftMotorVoltage().value();
    double right = logger.GetRightMotorVoltage().value();

    auto diffKine = frc::DifferentialDriveKinematics(1_m);

    if(doRotation){
        frc::DifferentialDriveWheelSpeeds velIn;
        velIn.left = logger.GetLeftMotorVoltage().value() * 1_mps;
        velIn.right = logger.GetRightMotorVoltage().value() * 1_mps;

        frc::ChassisSpeeds vel = diffKine.ToChassisSpeeds(velIn);
        swerve->setVoltage(vel);
    } else {
        swerve->setModules(tankSwerve(left, right), MotorControlMode::Voltage);
    }

    if(doRotation){
        auto swerveVel = swerve->getVelocity();
        frc::ChassisSpeeds vels;
        vels.vx = swerveVel.X();
        vels.vy = 0_mps;
        vels.omega = swerve->getYawRate() * 1_rad_per_s;
        auto wheelVels = diffKine.ToWheelSpeeds(vels);
        leftVel = wheelVels.left.value();
        rightVel = wheelVels.right.value();
    }

    logger.Log(
            leftPos, rightPos, leftVel, rightVel, yaw, angularRate
    );

    frc::SmartDashboard::GetEntry("Yaw").SetDouble(yaw);
}

void SwerveCharacterizationRobot::disabledInit() {
	swerve->setModules(tankSwerve(0, 0));
	logger.SendData();
}
