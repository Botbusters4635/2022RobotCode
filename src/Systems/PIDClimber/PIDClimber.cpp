//
// Created by cc on 29/01/22.
//

#include "PIDClimber.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

PIDClimber::PIDClimber(const PIDClimberConfig &config)
        : WPISubsystem("PIDClimber") {
    this->config = config;
    this->motors = config.motors;
//    this->config.springPID = springPID;
//    this->config.springLessPID = springLessPID;

    table = ntInstance.GetTable("PIDClimber");

    frc::ProfiledPIDController<units::meters>::Constraints constraints(
            units::meters_per_second_t(config.maxVelocity),
            units::meters_per_second_squared_t(config.maxAcceleration));

    pidController = std::make_unique<frc::ProfiledPIDController<units::meters>>(
            config.pidConfig.p, config.pidConfig.i, config.pidConfig.d, constraints);

    pidController->SetTolerance(units::meter_t(config.pidTolerance),
                                units::meters_per_second_t(0.05));

    if (motors.empty()) {
        log->error("No motors given to PIDClimber!!!");
        throw std::runtime_error("Motor vector is empty!");
    }


    if (config.isInverted.size() != motors.size())
        throw std::runtime_error("Motor inversion not configured!");

    for (size_t i = 0; i < motors.size(); i++) {
        const auto motor = motors[i];
        if (i == 0) {
            motor->enableBrakingOnIdle(true);
            motor->setMotorCurrentLimit(20);
            motor->enableCurrentLimit(true);
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(heightToRad(config.reverseSoftLimit));
            motor->setForwardSoftLimit(heightToRad(config.forwardSoftLimit));
            motor->enableForwardSoftLimit(true);
            motor->enableReverseSoftLimit(true);
            motor->enableLimitSwitches(false);
            motor->invertMotor(true);


        } else {
            motor->followMotor(*motors[0], config.isInverted[i]);
            motor->enableBrakingOnIdle(true);
            motor->setMotorCurrentLimit(20);
            motor->enableCurrentLimit(true);
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(heightToRad(config.reverseSoftLimit));
            motor->setForwardSoftLimit(heightToRad(config.forwardSoftLimit));
            motor->enableForwardSoftLimit(true);
            motor->enableReverseSoftLimit(true);
            motor->enableLimitSwitches(false);

        }
    }

    pidController->Reset(getHeight());
    setFF(PIDClimber::FeedForward::Free);

    masterMotorCurrent = table->GetEntry("masterMotorCurrent");
    slaveMotorCurrent = table->GetEntry("salveMotorCurrent");
}

void PIDClimber::robotInit() { ; }

void PIDClimber::setPID(const PIDConfig &pidConfig) {
    pidController->SetPID(pidConfig.p, pidConfig.i, pidConfig.d);
}

void PIDClimber::set(units::meter_t height) {
    pidController->SetGoal(height);
}

double PIDClimber::radToHeight(double rads) const {
    double motorRotations = rads / (2.0 * M_PI);
    motorRotations /= config.gearReduction;
    double height = motorRotations * (config.pulleyDiameter * M_PI);
    return height;
}

double PIDClimber::heightToRad(double height) const {
    double rots = height / (config.pulleyDiameter * M_PI);
    rots = config.gearReduction * rots;
    return rots * (2.0 * M_PI);
}

void PIDClimber::setVoltage(double setVoltage) { manualVoltage = setVoltage; }

bool PIDClimber::getLimitSwitch() const {
    return motors[0]->getForwardLimitSwitch();
}

units::meters_per_second_t PIDClimber::getVelocity() const {
    auto encVel = motors[0]->getVelocity();
    return units::meters_per_second_t(radToHeight(encVel));
}

void PIDClimber::usePIDControl(bool usePID) {
    if (this->usePID != usePID && usePID) {
        resetController();
    }
    this->usePID = usePID;
}

void PIDClimber::resetToZero() {
    motors[0]->setSensorPosition(0);
}

void PIDClimber::resetController() {
    pidController->Reset(getHeight(), getVelocity());
}

bool PIDClimber::isAtGoal() { return pidController->AtGoal(); }

void PIDClimber::setFF(FeedForward ff) {
    switch (ff) {
        case PIDClimber::FeedForward::Free:
            this->ff = &this->config.freeFF;
            break;
        case PIDClimber::FeedForward::Loaded:
            this->ff = &this->config.loadedFF;
            break;
        default:

            break;
    }
}

void PIDClimber::useSoftLimits(bool set) {
    motors[0]->enableForwardSoftLimit(set);
    motors[0]->enableReverseSoftLimit(set);
}

units::meter_t PIDClimber::getHeight() const {
    double motorRads = motors[0]->getPosition();
    return units::meter_t(radToHeight(motorRads));
}

void PIDClimber::robotUpdate() {
//    if (getHeight() > 0.63_m){
//        setPID(springLessPID);
//        setFF(FeedForward::Loaded);
//        frc::SmartDashboard::PutString("climber", "springless");
//    }else{
//        setPID(springPID);
//        setFF(FeedForward::Free);
//        frc::SmartDashboard::PutString("climber", "springed");
//    }
    double out;
    out = pidController->Calculate(getHeight());
    out += ff->Calculate(pidController->GetSetpoint().velocity).value();
    motors[0]->set(out, MotorControlMode::Voltage);


    table->GetEntry("Goal")
            .SetDouble(pidController->GetGoal().position.value());

    table->GetEntry("VoltageApplied").SetDouble(out);
    frc::SmartDashboard::PutData("PIDClimber/PIDClimber",
                                 pidController.get());

//    masterMotorCurrent.SetDouble(motors[0]->getCurrent());
//    slaveMotorCurrent.SetDouble(motors[1]->getCurrent());

    table->GetEntry("ClimberPosition").SetDouble(getHeight().value());
    table->GetEntry("ClimberLimitSwitch").SetBoolean(getLimitSwitch());
    table->GetEntry("UsePID").SetBoolean(usePID);
}
