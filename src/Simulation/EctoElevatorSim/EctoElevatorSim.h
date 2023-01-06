//
// Created by abiel on 2/28/22.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOELEVATORSIM_H
#define BOTBUSTERS_REBIRTH_ECTOELEVATORSIM_H

#include "Core/EctoModule/Manager.h"
#include "Core/MotorHandler/EctoMotor/EctoSimulatedMotor.h"
#include <frc/simulation/ElevatorSim.h>

class EctoElevatorSim : public Manager{
public:
    EctoElevatorSim(const std::shared_ptr<EctoSimulatedMotor> &ectoMotor, const frc::DCMotor& motor, units::kilogram_t m, units::meter_t r, double G);

    void init() override;

    void update() override;
private:
    std::unique_ptr<frc::sim::ElevatorSim> sim;
    std::shared_ptr<EctoSimulatedMotor> motor;
    units::meter_t r;
    double G;
};


#endif //BOTBUSTERS_REBIRTH_ECTOELEVATORSIM_H
