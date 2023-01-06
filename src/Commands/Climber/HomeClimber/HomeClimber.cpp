//
// Created by cc on 28/01/22.
//

#include "HomeClimber.h"

#include <frc2/command/PrintCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/StartEndCommand.h>

#ifdef SIMULATION
#include <frc2/command/WaitCommand.h>
#endif

HomeClimber::HomeClimber(const std::shared_ptr<PIDClimber> &climber) {
    auto homingSequence = [climber](double voltage) {
        return frc2::FunctionalCommand(
                [climber] {
                    climber->useSoftLimits(false);
                    climber->usePIDControl(false);
                },//Init
                [climber, voltage] { climber->setVoltage(voltage); }, //Execute
                [climber](bool interrupted) {
                    if (!interrupted) {
                        climber->resetToZero();
                        climber->set(0);
                    }

                    climber->usePIDControl(true);
                    climber->useSoftLimits(true);
                }, //End
                [climber] {
                    return climber->getLimitSwitch();
                }, //HasFinished
                {climber.get()}
        );
    };

    auto manualMove = [climber](double voltage) {
        return frc2::StartEndCommand(
                [climber, voltage] {
                    climber->usePIDControl(false);
                    climber->setVoltage(voltage);
                },
                [climber] {
                    climber->setVoltage(0);
                    climber->usePIDControl(true);
                }
        );
    };

    AddCommands(
            frc2::PrintCommand("Starting home climber"),
            homingSequence(-0.75), //Home at (relatively) high vel
            manualMove(1).WithTimeout(units::second_t(0.5)), //Move up for 0.5 sec
            homingSequence(-0.75), //Home again at slower velocity
            frc2::InstantCommand([climber = climber]{
                climber->setClimberHomed(true);
            }),
            frc2::PrintCommand("Ending home climber")

    );
}