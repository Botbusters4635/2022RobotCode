//
// Created by abiel on 4/9/22.
//

#include "DelayedBallShoot.h"
#include <frc/Timer.h>
#include "Commands/Shooter/Preload/PreloadBalls.h"
#include "Commands/Shooter/ShootNBalls/ShootNBalls.h"
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>

DelayedBallShoot::DelayedBallShoot(const std::shared_ptr<Feeder> &feeder, units::second_t delay) {
    AddCommands(
                ShootNBalls(feeder, 1),
                frc2::ParallelDeadlineGroup(
                        frc2::WaitCommand(delay),
                        PreloadBalls(feeder)
                        )
            );
}
