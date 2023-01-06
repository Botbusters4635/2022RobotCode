//
// Created by 4ndre on 04/10/2021.
//

#include "PIDShooter.h"
#include <frc/Timer.h>
#include "Math/EctoMath.h"
PIDShooter::PIDShooter(const PIDShooterConfig &config) : Shooter("PIDShooter", {0,1}),
                                                         pidController(config.pidConfig.p, config.pidConfig.i, config.pidConfig.d),
                                                         slewRateLimiter(config.maxAcceleration){
    this->config = config;
    this->motors = config.motors;
    this->ff = config.ff;

    if(this->motors.empty()){
        log->error("No motors given to PIDShooter!!!");
        throw std::runtime_error("Motor vector is empty!");
    }

    table = ntInstance.GetTable("PIDShooter");
    
    motors[0]->setControlMode(MotorControlMode::Voltage);
    motors[0]->setPIDConfig(config.pidConfig, 0);
    motors[0]->setClosedLoopRampRate(config.rampRate);
    motors[0]->setMotorCurrentLimit(config.currentLimit);
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
        }
    }

    ntPIDConfig = std::make_unique<NetworkTablePID>("PIDShooter/PIDConfig", config.pidConfig, [this](const PIDConfig &config){
        this->setPIDConfig(config);
    });

    slewRateLimiter.Reset(units::radians_per_second_t(0));
    pidController.Reset();
    pidController.SetTolerance(10);
    slewRateLimiter.Calculate(0_rad_per_s);
}

void PIDShooter::robotInit() {
   ;
}

void PIDShooter::robotUpdate() {
    const double velocity = getVelocity();
    const double velSetpoint = slewRateLimiter.Calculate(setpoint).value();

    double pidOut = pidController.Calculate(velocity, velSetpoint);
    double ffOut = config.ff.Calculate(units::radians_per_second_t (velSetpoint)).value(); //V
    double out = pidOut + ffOut;

    motors[0]->set(out);

    table->GetEntry("Setpoint").SetDouble(velSetpoint);
    table->GetEntry("Goal").SetDouble(setpoint.value());
    table->GetEntry("Velocity").SetDouble(velocity);
    table->GetEntry("AppliedVoltage").SetDouble(out);
    table->GetEntry("motor 0").SetDouble(motors[0]->getCurrent());
    table->GetEntry("motor 1").SetDouble(motors[1]->getCurrent());
}

void PIDShooter::setPIDConfig(const PIDConfig &pidConfig) {
    this->config.pidConfig = pidConfig;
    motors[0]->setPIDConfig(pidConfig, 0);
}

double PIDShooter::getVelocity() const {
    return (motors[0]->getVelocity() / config.gearReduction);
}

void PIDShooter::setSetpoint(double velocity) {
    setpoint = units::radians_per_second_t(velocity);
}

bool PIDShooter::flywheelAtSetpoint() {
    return pidController.AtSetpoint();
}