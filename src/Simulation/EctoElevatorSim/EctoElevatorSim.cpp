//
// Created by abiel on 2/28/22.
//

#include "EctoElevatorSim.h"
#include <frc/system/plant/LinearSystemId.h>

EctoElevatorSim::EctoElevatorSim(const std::shared_ptr<EctoSimulatedMotor> &ectoMotor, const frc::DCMotor &motor,
                                 units::kilogram_t m, units::meter_t r, double G) : Manager("EctoElevatorSim") {
    sim = std::make_unique<frc::sim::ElevatorSim>(
            motor,
            G,
            m,
            r,
            0.0_m,
            1.0_m);
    this->motor = ectoMotor;
    this->r = r;
    this->G = G;
}

void EctoElevatorSim::init() {
    ;
}

void EctoElevatorSim::update() {
    sim->SetInputVoltage(motor->getOutputVoltage());
    sim->Update(20_ms);
    motor->setPosition(sim->GetPosition().value() / (r.value() * M_PI) * G * (2.0 * M_PI));
    motor->setVelocity(sim->GetVelocity().value() / (2.0 * M_PI) * r.value());
    motor->setCurrentDraw(sim->GetCurrentDraw().value());
}