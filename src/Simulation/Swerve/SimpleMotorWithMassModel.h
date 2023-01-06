//
// Created by abiel on 2/4/22.
//

#ifndef BOTBUSTERS_REBIRTH_SIMPLEMOTORWITHMASSMODEL_H
#define BOTBUSTERS_REBIRTH_SIMPLEMOTORWITHMASSMODEL_H

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/FlywheelSim.h>
#include <units/angular_velocity.h>

class SimpleMotorWithMassModel {
public:
    SimpleMotorWithMassModel();
    SimpleMotorWithMassModel(const frc::DCMotor &motor, double gearing, units::kilogram_square_meter_t moi);

    void update(units::volt_t motorVoltage, units::second_t dt);

    units::revolutions_per_minute_t  getMechanismSpeed() const;

    units::ampere_t getCurrent() const;

    double getMechanisimPositionRev() const;

private:
    units::radian_t m_curDisplacement{0};
    frc::sim::FlywheelSim m_fwSim;
};


#endif //BOTBUSTERS_REBIRTH_SIMPLEMOTORWITHMASSMODEL_H
