//
// Created by abiel on 2/4/22.
//

#ifndef BOTBUSTERS_REBIRTH_MOTORGEARBOXWHEELSIM_H
#define BOTBUSTERS_REBIRTH_MOTORGEARBOXWHEELSIM_H

#include <frc/system/plant/DCMotor.h>
#include <units/velocity.h>
#include <units/torque.h>
#include <units/angular_velocity.h>

class MotorGearboxWheelSim {
public:
    MotorGearboxWheelSim();

    MotorGearboxWheelSim(const frc::DCMotor &motor,
                         double gearRatio,
                         units::meter_t wheelRadius,
                         double gearboxFricCoefNmPerRadPerSec);

    void update(units::meters_per_second_t groundVelocity,
                units::volt_t motorVoltage,
                units::second_t dt);

    double getPositionRev() const;

    units::newton_t getGroundForce() const;

    units::revolutions_per_minute_t getWheelSpeed() const;

    units::revolutions_per_minute_t getMotorSpeed() const;
private:
    frc::DCMotor m_motor;
    double m_gearRatio;
    units::meter_t m_wheelRadius;
    units::newton_t m_curGroundForce;
    units::radian_t m_wheelRotations;

    double m_gearboxFricCoefNmPerRadPerSec;
    units::radians_per_second_t m_prevWheelRotationalSpeed;

    units::revolutions_per_minute_t m_wheelSpeed;
    units::revolutions_per_minute_t m_motorSpeed;
};


#endif //BOTBUSTERS_REBIRTH_MOTORGEARBOXWHEELSIM_H
