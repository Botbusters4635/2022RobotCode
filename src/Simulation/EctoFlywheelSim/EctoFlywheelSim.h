//
// Created by abiel on 2/8/22.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOFLYWHEELSIM_H
#define BOTBUSTERS_REBIRTH_ECTOFLYWHEELSIM_H

#include "Core/EctoModule/Manager.h"
#include "Core/MotorHandler/EctoMotor/EctoSimulatedMotor.h"
#include <frc/simulation/FlywheelSim.h>

class EctoFlywheelSim : public Manager {
public:
    EctoFlywheelSim(const std::shared_ptr<EctoSimulatedMotor> &ectoMotor, const frc::DCMotor& motor, double G, double kV, double kA);

    void init() override;

    void update() override;
private:
    std::unique_ptr<frc::sim::FlywheelSim> sim;
    std::shared_ptr<EctoSimulatedMotor> motor;
};


#endif //BOTBUSTERS_REBIRTH_ECTOFLYWHEELSIM_H
