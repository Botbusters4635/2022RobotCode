//
// Created by cc on 19/05/22.
//

#include "HomeTurret.h"


#ifdef SIMULATION
#include <frc2/command/WaitCommand.h>
#endif

HomeTurret::HomeTurret(const std::shared_ptr<PIDTurret> &turret) {
    auto homingSequence = [turret](double voltage){
        return frc2::FunctionalCommand(
        [turret]{
            turret->useSoftLimits(false);
            turret->usePIDControl(false);

        },//init
        [turret, voltage] {turret->setVoltage(voltage);},//execute
        [turret](bool interrupted){
            if (!interrupted){
                turret->resetToZero();
                turret->set(0);
            }
//            turret->usePIDControl(true);
            turret->useSoftLimits(true);
        }, //end
        [turret] {
            return turret->getReverseLimitSwitch();
        }, //hasFinished
                {turret.get()}
        );
    };
    auto manualMove = [turret](double voltage){
        return frc2::StartEndCommand(
                [turret, voltage]{
                    turret->usePIDControl(false);
                    turret->useSoftLimits(false);
                    turret->setVoltage(voltage);
                },
                [turret]{
                    turret->setVoltage(0);
                    turret->usePIDControl(true);
                    turret->useSoftLimits(true);
                }
        );
    };

    AddCommands(
            frc2::PrintCommand("started Turret Homing sequence"),
            homingSequence(-0.75),
            manualMove(1).WithTimeout(0.2_s),
            homingSequence(-0.5),
            frc2::InstantCommand([turret = turret]{
                turret->usePIDControl(true);
                turret->setTurretHomed(true);
                turret->setConfigCurrentLimit();
            }),
            frc2::PrintCommand("ended Homing sequence")
    );
}