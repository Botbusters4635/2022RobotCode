//
// Created by abiel on 2/26/22.
//

#include "SwerveCharacterizationRotation.h"

SwerveCharacterizationRotation::SwerveCharacterizationRotation() : EctoCharacterizationRobot("SwerveCharacterizationRotation") {
    ;
}

void SwerveCharacterizationRotation::robotInit() {
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

void SwerveCharacterizationRotation::teleopUpdate() {
    swerve->setModules(tankSwerve(leftSide.get() * 12.0, rightSide.get() * 12.0), MotorControlMode::Voltage);
}

void SwerveCharacterizationRotation::autoInit() {
    lastRunTime = frc::Timer::GetFPGATimestamp();
    lastYaw = swerve->getRawYaw();
    yawRateFilter.Reset(0_rad_per_s);
    logger.InitLogging();
}

void SwerveCharacterizationRotation::autoUpdate() {
    auto dt = frc::Timer::GetFPGATimestamp() - lastRunTime;
    auto state = swerve->getMotorStates();

    auto leftPos = (state.topLeft.wheelPosition + state.backLeft.wheelPosition) / 2.0;
    auto rightPos = (state.topRight.wheelPosition + state.backRight.wheelPosition) / 2.0;

    auto leftVel = (state.topLeft.wheelVelocity + state.backLeft.wheelVelocity) / 2.0;
    auto rightVel = (state.topRight.wheelVelocity + state.backRight.wheelVelocity) / 2.0;

    auto yaw = swerve->getRawYaw();
    //auto angularRate = (yaw - lastYaw) / dt.value();
    auto angularRate = swerve->getYawRate();
//    if(yaw == lastYaw){
//        angularRate = lastAngularRate;
//    } else {
//        angularRate = yawRateFilter.Calculate(units::radians_per_second_t (angularRate)).value();
//    }

    double voltage = logger.GetMotorVoltage().value();
    auto states = swerve->getMotorStates().toWPI();
    auto vel = swerve->getKinematics()->ToChassisSpeeds(states);

    logger.Log(
            yaw, vel.omega.value()
    );

    frc::ChassisSpeeds speeds;
    speeds.vx = 0_mps;
    speeds.vy = 0_mps;
    speeds.omega = units::radians_per_second_t (voltage);
    swerve->setVoltage(speeds);
    lastRunTime = frc::Timer::GetFPGATimestamp();
    lastYaw = yaw;
    lastAngularRate = angularRate;
}

void SwerveCharacterizationRotation::disabledInit() {
    swerve->setModules(tankSwerve(0, 0));
    logger.SendData();
}