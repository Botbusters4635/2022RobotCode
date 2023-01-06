//
// Created by cc on 11/05/22.
//

#include "PIDTurret.h"
#include <frc/smartdashboard/SmartDashboard.h>

PIDTurret::PIDTurret(const PIDTurretConfig &config) : WPISubsystem("turret") {
    this->config = config;
    this->motor = config.turretMotor;

//    motor->setControlMode(config.motorControlMode);
    motor->setControlMode(MotorControlMode::Voltage);
    motor->setMotorCurrentLimit(config.currentLimit);
    motor->enableCurrentLimit(true);
    motor->enableBrakingOnIdle(true);
    motor->setClosedLoopRampRate(config.rampRate);
    motor->invertMotor(config.isInverted);
    motor->setFeedbackMode(MotorFeedbackMode::QuadEncoder);
    motor->setForwardSoftLimit(config.forwardSoftLimit.value() * config.gearReduction);
    motor->setReverseSoftLimit(config.reverseSoftLimit.value() * config.gearReduction);
    motor->enableForwardSoftLimit(config.enableForwardSoftLimit);
    motor->enableReverseSoftLimit(config.enableReverseSoftLimit);

    frc::ProfiledPIDController<units::radians>::Constraints constraints{config.maxVel, config.maxAccel};

    pidController = std::make_unique<frc::ProfiledPIDController<units::radians>>(
            config.pidConfig.p, config.pidConfig.i, config.pidConfig.d, constraints);
    pidController->SetTolerance(config.pidDistTol, config.pidVelTol);
    pidController->Reset(getHeading());


    this->ff = &this->config.ff;

    table = ntInstance.GetTable("Turret");
    table->GetEntry("testing/TurretHeading").SetDouble(0);

    setToRobot(0_rad);


}

void PIDTurret::robotInit() {;}


units::radian_t PIDTurret::getHeading() {
//    mutex.lock();
    auto pose = units::radian_t(motor->getPosition() / config.gearReduction);
//    mutex.unlock();
    return pose;
}

units::radians_per_second_t PIDTurret::getVel() {
    return units::radians_per_second_t (motor->getVelocity() / config.gearReduction);
}

bool PIDTurret::getForwardLimitSwitch() {
    return motor->getForwardLimitSwitch();
}

bool PIDTurret::getReverseLimitSwitch() {
    return motor->getReverseLimitSwitch();
}

void PIDTurret::resetToZero() {
    motor->setSensorPosition(0);
}

void PIDTurret::setSensorAngle(units::radian_t angle) {
    motor->setSensorPosition(angle.value() * config.gearReduction);
}

void PIDTurret::useSoftLimits(bool useSoftLimits){
    motor->enableForwardSoftLimit(useSoftLimits);
    motor->enableReverseSoftLimit(useSoftLimits);
    table->GetEntry("useSoftLimits").SetBoolean(useSoftLimits);
}

void PIDTurret::usePIDControl(bool usePID) {
    if (this->usePID != usePID && usePID){
        resetController();
    }
    this->usePID = usePID;
}

void PIDTurret::set(units::radian_t radians) {
    frc::SmartDashboard::PutNumber("whatIsUpWithTurret/setIn", radians.value());
    auto goal = units::radian_t(deWrapAngle(radians.value()));
    pidController->SetGoal(goal);
    inDeadZone = goal > config.forwardSoftLimit || goal < config.reverseSoftLimit;
}

void PIDTurret::setToRobot(units::radian_t radians) {
    set(radians - config.turretZeroOffsetToRobot);
}

void PIDTurret::resetController() {
    pidController->Reset(getHeading(), getVel());
}

void PIDTurret::setPID(PIDConfig &pidConfig) {
    pidController->SetPID(pidConfig.p, pidConfig.i, pidConfig.d);
}

bool PIDTurret::atGoal(){return pidController->AtGoal();}

double PIDTurret::deWrapAngle(double radians) {
    double out = 0.0;
    radians = EctoMath::wrapAngle(radians);
    if (radians < EctoMath::degreesToRadians(-7.5)){
        radians = radians + (2 * M_PI);
    }
    out = std::clamp(radians, 0.0, EctoMath::degreesToRadians(345));
    return out;
}

frc::Transform2d PIDTurret::getCameraToRobot() {
    std::pair<double, double> xy{EctoMath::polarToCartesian(config.turretToCamera.value(), getHeadingToRobot().Radians().value())};
//    std::lock_guard<std::mutex> lock(turretMutex);
    return {{units::meter_t(xy.first) + config.turretCenterToRobotCenter.X(),
             units::meter_t(xy.second) + config.turretCenterToRobotCenter.Y()},
            getHeadingToRobot()};

}

frc::Transform2d PIDTurret::simGetCameraToRobot() {
    std::pair<double, double> xy{EctoMath::polarToCartesian(config.turretToCamera.value(), simGetHeadingToRobot().Radians().value())};

    return {{units::meter_t(xy.first) + config.turretCenterToRobotCenter.X(),
             units::meter_t(xy.second) + config.turretCenterToRobotCenter.Y()},
            simGetHeadingToRobot()};
}

units::volt_t PIDTurret::calculateFF(units::radians_per_second_t vel) {
    return ff->Calculate(vel);
}

void PIDTurret::robotUpdate() {
    double out;
    if (usePID) {
        out = pidController->Calculate(getHeading());
        out += ff->Calculate(pidController->GetSetpoint().velocity).value();
        motor->set(out, MotorControlMode::Voltage);
    } else {
        out = manualVoltage;
        motor->set(out, MotorControlMode::Voltage);
    }


    table->GetEntry("Goal").SetDouble(out);
    table->GetEntry("Heading").SetDouble(getHeading().value());
    table->GetEntry("Vel").SetDouble(getVel().value());
    table->GetEntry("usePID").SetBoolean(usePID);
    table->GetEntry("manualVoltage").SetDouble(manualVoltage);
    table->GetEntry("forwardLimitSwitch").SetBoolean(motor->getForwardLimitSwitch());
    table->GetEntry("reverseLimitSwitch").SetBoolean(motor->getReverseLimitSwitch());
    table->GetEntry("current").SetDouble(motor->getCurrent());
    table->GetEntry("HeadingToRobot").SetDouble(getHeadingToRobot().Radians().value());
    table->GetEntry("turretPose/X").SetDouble(getCameraToRobot().X().value());
    table->GetEntry("turretPose/Y").SetDouble(getCameraToRobot().Y().value());
    table->GetEntry("turretPose/heading").SetDouble(getCameraToRobot().Rotation().Degrees().value());

    simHeading = table->GetEntry("testing/TurretHeading").GetDouble(0);

}


