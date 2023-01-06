//
// Created by andrew on 10/11/21.
//

#include "LQRShooter.h"
#include <frc/system/plant/LinearSystemId.h>

LQRShooter::LQRShooter(const ShooterLQRConfig &config) : Shooter("LQRShooter", {2,3, 8}),
                                                         slewRateLimiter(config.maxAcceleration),
                                                         flywheelPlant(frc::LinearSystemId::IdentifyVelocitySystem<units::radian>(config.kV / 1_rad_per_s,
                                                                                                                                  config.kA / 1_rad_per_s_sq)),
                                                         observer(flywheelPlant,
                                                                  {6},
                                                                  {1},
                                                                  20_ms),
                                                         controller(flywheelPlant,
                                                                    {0.01},
                                                                    {12.0},
                                                                    20_ms),
                                                        loop(flywheelPlant, controller,
                                                                         observer, 12_V, 20_ms){
    table = ntInstance.GetTable("ShooterLQR");

    this->config = config;
    this->motors = config.motors;


    motors[0]->setControlMode(MotorControlMode::Voltage);
    motors[0]->setMotorCurrentLimit(30.0);
    motors[0]->enableCurrentLimit(true);
    motors[0]->enableBrakingOnIdle(false);
    motors[0]->setCANTimeout(5);

    if(config.isInverted.size() != motors.size())
        throw std::runtime_error("Motor inversion not configured!");

    for(size_t i = 0; i < motors.size(); i++){
        const auto motor = motors[i];
        if(i == 0) {
            motor->invertMotor(config.isInverted[i]);
        } else {
            motor->followMotor(*motors[0], config.isInverted[i]);
            motor->deprioritizeUpdateRate();
            motor->setMotorCurrentLimit(25.0);
            motor->enableCurrentLimit(true);
        }
    }

    slewRateLimiter.Reset(0_rad_per_s);
    slewRateLimiter.Calculate(0_rad_per_s);

    setHood(0);

    controller.LatencyCompensate(flywheelPlant, 20_ms, 25_ms);
    loop = frc::LinearSystemLoop<1,1,1>(flywheelPlant, controller,
                                        observer, 12_V, 20_ms);
    loop.Reset(Eigen::Vector<double,1>{0.0});
}

void LQRShooter::robotInit() {
    ;
}

void LQRShooter::robotUpdate() {
    auto setpoint = slewRateLimiter.Calculate(targetSetpoint);

    table->GetEntry("Velocity").SetDouble(getVelocity());
    table->GetEntry("Setpoint").SetDouble(setpoint.value());
    table->GetEntry("MasterCurrent").SetDouble(motors[0]->getCurrent());
    table->GetEntry("SlaveCurrent").SetDouble(motors[1]->getCurrent());


    loop.SetNextR(Eigen::Vector<double,1>{std::clamp(setpoint.value(), 0.0, 550.0)});
    loop.Correct(Eigen::Vector<double,1>{getVelocity()});
    loop.Predict(20_ms);

    motors[0]->set(loop.U(0), MotorControlMode::Voltage);
}

double LQRShooter::getVelocity() const {
    return motors[0]->getVelocity() / config.gearRatio;
}

void LQRShooter::setSetpoint(double velocity) {
    targetSetpoint = units::radians_per_second_t(velocity);
}

bool LQRShooter::flywheelAtSetpoint() {
    return units::math::abs(targetSetpoint - units::radians_per_second_t(getVelocity())) < 6_rad_per_s;
}
