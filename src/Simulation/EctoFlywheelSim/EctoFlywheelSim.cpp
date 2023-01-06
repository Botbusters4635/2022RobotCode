//
// Created by abiel on 2/8/22.
//

#include "EctoFlywheelSim.h"
#include <frc/system/plant/LinearSystemId.h>

EctoFlywheelSim::EctoFlywheelSim(const std::shared_ptr<EctoSimulatedMotor> &ectoMotor, const frc::DCMotor &motor, double G,
                                 double kV, double kA) : Manager("EctoFlywheelSim") {
    auto flywheelSystem = frc::LinearSystemId::IdentifyVelocitySystem<units::radians>(
            units::volt_t(kV) / 1_rad_per_s,
            units::volt_t(kA) / 1_rad_per_s_sq);
    sim = std::make_unique<frc::sim::FlywheelSim>(flywheelSystem, motor, G);
    this->motor = ectoMotor;
}

void EctoFlywheelSim::init() {
    ;
}

void EctoFlywheelSim::update() {
    sim->SetInputVoltage(motor->getOutputVoltage());
    sim->Update(units::millisecond_t(20));
    motor->setVelocity(sim->GetAngularVelocity().value());
    motor->setCurrentDraw(sim->GetCurrentDraw().value());
}