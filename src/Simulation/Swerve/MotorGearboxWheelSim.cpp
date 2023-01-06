//
// Created by abiel on 2/4/22.
//

#include "MotorGearboxWheelSim.h"

MotorGearboxWheelSim::MotorGearboxWheelSim() : m_motor(frc::DCMotor::NEO()){;}

MotorGearboxWheelSim::MotorGearboxWheelSim(const frc::DCMotor &motor, double gearRatio, units::meter_t wheelRadius,
                                           double gearboxFricCoefNmPerRadPerSec) : m_motor(motor) {
    m_gearRatio = gearRatio;
    m_wheelRadius = wheelRadius;
    m_gearboxFricCoefNmPerRadPerSec = gearboxFricCoefNmPerRadPerSec;
}

void MotorGearboxWheelSim::update(units::meters_per_second_t groundVelocity, units::volt_t motorVoltage,
                                  units::second_t dt) {
    units::radians_per_second_t wheelRotSpd = units::radians_per_second_t (groundVelocity.value() / m_wheelRadius.value());
    units::radians_per_second_t motorRotSpd = wheelRotSpd * m_gearRatio;

    units::newton_meter_t motorTq = m_motor.Kt * m_motor.Current(motorRotSpd, motorVoltage);

    auto gboxFricTq = units::newton_meter_t(motorRotSpd.value() * m_gearboxFricCoefNmPerRadPerSec);
    auto curWheelTq = motorTq * m_gearRatio - gboxFricTq;

    m_curGroundForce = curWheelTq / m_wheelRadius / 2.0;

    m_wheelRotations += (wheelRotSpd + m_prevWheelRotationalSpeed) / 2.0 * dt;

    m_prevWheelRotationalSpeed = wheelRotSpd;

    m_wheelSpeed = wheelRotSpd;
    m_motorSpeed = motorRotSpd;
}

double MotorGearboxWheelSim::getPositionRev() const {
    return m_wheelRotations.value() / 2.0 / M_PI;
}

units::newton_t MotorGearboxWheelSim::getGroundForce() const {
    return m_curGroundForce;
}

units::revolutions_per_minute_t MotorGearboxWheelSim::getWheelSpeed() const {
    return m_wheelSpeed;
}

units::revolutions_per_minute_t MotorGearboxWheelSim::getMotorSpeed() const {
    return m_motorSpeed;
}