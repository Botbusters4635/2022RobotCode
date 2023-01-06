//
// Created by cc on 12/06/22.
//

#include "LQRTurret.h"
#include <frc/system/plant/LinearSystemId.h>
#include <frc/smartdashboard/SmartDashboard.h>

LQRTurret::LQRTurret(const LQRTurretConfig &config) : WPISubsystem("LQRTurret"),
                                                      velLimiter(config.maxVel),
                                                      accelLimiter(config.maxAccel),
                                                      turretPlant(frc::LinearSystemId::IdentifyPositionSystem<units::radian>(config.kV / 1_rad_per_s,
                                                                                                                             config.kA / 1_rad_per_s_sq)),
                                                      observer(turretPlant,
                                                               {0.1, 1.0},
                                                               {0.1},
                                                               20_ms),
                                                      controller(turretPlant,
                                                                 {0.03, 0.3},
                                                                 {config.maxControlVoltage.value()},
                                                                 20_ms),
                                                      loop(turretPlant, controller, observer, config.maxControlVoltage, 20_ms),
                                                      constraints(config.maxVel, config.maxAccel){
    table = ntInstance.GetTable("LQRTurret");
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

    table = ntInstance.GetTable("Turret");
    table->GetEntry("testing/TurretHeading").SetDouble(0);

    setToRobot(0_rad);

    lastProfiledReference = {getHeading(), getVel()};

    controller.LatencyCompensate(turretPlant, 20_ms, 25_ms);
    loop = frc::LinearSystemLoop<2,1,1>(turretPlant, controller, observer, 12_V, 20_ms);

    loop.Reset(Eigen::Vector<double, 2>{0.0, 0.0});

    lastProfiledReference = {getHeading(), getVel()};




}

void LQRTurret::set(units::radian_t radians) {
    auto goal = units::radian_t(deWrapAngle(radians.value()));
    targetSetpoint = goal;
}

units::radian_t LQRTurret::getHeading() {
//    mutex.lock();
    auto pose = units::radian_t(motor->getPosition() / config.gearReduction);
//    mutex.unlock();
    return pose;
}

units::radians_per_second_t LQRTurret::getVel() {
    return units::radians_per_second_t (motor->getVelocity() / config.gearReduction);
}

double LQRTurret::deWrapAngle(double radians) {
    double out = 0.0;
    radians = EctoMath::wrapAngle(radians);
    if (radians < EctoMath::degreesToRadians(-7.5)){
        radians = radians + (2 * M_PI);
    }
    out = std::clamp(radians, 0.0, EctoMath::degreesToRadians(345));
    return out;
}

bool LQRTurret::getForwardLimitSwitch() {
    return motor->getForwardLimitSwitch();
}

bool LQRTurret::getReverseLimitSwitch() {
    return motor->getReverseLimitSwitch();
}

void LQRTurret::resetToZero() {
    motor->setSensorPosition(0);
}

void LQRTurret::setSensorAngle(units::radian_t angle) {
    motor->setSensorPosition(angle.value() * config.gearReduction);
}

void LQRTurret::useSoftLimits(bool useSoftLimits){
    motor->enableForwardSoftLimit(useSoftLimits);
    motor->enableReverseSoftLimit(useSoftLimits);
    table->GetEntry("useSoftLimits").SetBoolean(useSoftLimits);
}

void LQRTurret::setToRobot(units::radian_t radians) {
    set(radians - config.turretZeroOffsetToRobot);
}

//void PIDTurret::resetController() {
//    loop.Reset(getHeading());
//}
frc::Transform2d LQRTurret::getCameraToRobot() {
    std::pair<double, double> xy{EctoMath::polarToCartesian(config.turretToCamera.value(), getHeadingToRobot().Radians().value())};
//    std::lock_guard<std::mutex> lock(turretMutex);
    return {{units::meter_t(xy.first) + config.turretCenterToRobotCenter.X(),
             units::meter_t(xy.second) + config.turretCenterToRobotCenter.Y()},
            getHeadingToRobot()};

}

void LQRTurret::robotInit(){ ; }

void LQRTurret::updateTelemetry() {
    table->GetEntry("velocity").SetDouble(getVel().value());
    table->GetEntry("setPoint").SetDouble(targetSetpoint.value());
    table->GetEntry("heading").SetDouble(getHeading().value());
}

void LQRTurret::robotUpdate(){
//    auto setPoint = velLimiter.Calculate(targetSetpoint);
    frc::TrapezoidProfile<units::radians>::State goal;

    goal = {targetSetpoint, 0_rad_per_s};

    lastProfiledReference =
            (frc::TrapezoidProfile<units::radians>(constraints, goal,
                                                   lastProfiledReference))
                    .Calculate(20_ms);
    frc::SmartDashboard::PutNumber("lastProfiledReference/Pose",lastProfiledReference.position.value());
    frc::SmartDashboard::PutNumber("lastProfiledReference/vel",lastProfiledReference.velocity.value());


    loop.SetNextR(Eigen::Vector<double, 2>{lastProfiledReference.position.value(),
                                           lastProfiledReference.velocity.value()});
    loop.Correct(Eigen::Vector<double ,1>{getHeading().value()});
    loop.Predict(20_ms);

    frc::SmartDashboard::PutNumber("setPoint", loop.U(0));

    motor->set(loop.U(0), MotorControlMode::Voltage);

}



