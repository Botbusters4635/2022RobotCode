//
// Created by abiel on 2/4/22.
//

#include "SimpleMotorWithMassModel.h"

SimpleMotorWithMassModel::SimpleMotorWithMassModel() : m_fwSim(frc::DCMotor::NEO(), 1, units::kilogram_square_meter_t(1)) {
    ;
}

SimpleMotorWithMassModel::SimpleMotorWithMassModel(const frc::DCMotor &motor, double gearing,
                                                   units::kilogram_square_meter_t moi) :
        m_fwSim(motor, gearing, moi) {
    ;
}

void SimpleMotorWithMassModel::update(units::volt_t motorVoltage, units::second_t dt) {
    m_fwSim.SetInputVoltage(motorVoltage);
    m_fwSim.Update(dt);

    m_curDisplacement += m_fwSim.GetAngularVelocity() * dt;
}

units::revolutions_per_minute_t SimpleMotorWithMassModel::getMechanismSpeed() const {
    return units::convert<units::rad_per_s, units::revolutions_per_minute>(m_fwSim.GetAngularVelocity());
}

units::ampere_t SimpleMotorWithMassModel::getCurrent() const {
    return m_fwSim.GetCurrentDraw();
}

double SimpleMotorWithMassModel::getMechanisimPositionRev() const {
    return m_curDisplacement / units::radian_t(2.0 * M_PI);
}
