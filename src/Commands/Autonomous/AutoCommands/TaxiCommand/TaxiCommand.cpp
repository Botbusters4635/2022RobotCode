//
// Created by abiel on 4/15/22.
//

#include "TaxiCommand.h"
#include <frc2/command/ParallelRaceGroup.h>
#include "Commands/Autonomous/AutoCommands/ResetOdometryToASetPoint/ResetOdoToSetPoint.h"
#include "Commands/Autonomous/AutoCommands/VisionAlign/VisionAlign.h"
#include "Commands/Shooter/ShootNBalls/ShootNBalls.h"

TaxiCommand::TaxiCommand(const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<VisionManager> &visionManager,
                         const std::shared_ptr<Feeder> &feeder, const frc::Pose2d &initialPose,
                         const frc::ChassisSpeeds &targetVoltage, units::second_t waitTime, units::second_t runTime){
    AddRequirements(swerve.get());

    AddCommands(
            ResetOdoToSetPoint(swerve, initialPose),
            frc2::WaitCommand(waitTime),
            frc2::InstantCommand([swerve, targetVoltage]{
                auto newVel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(targetVoltage.vx, targetVoltage.vy, targetVoltage.omega, swerve->getRotation());
                swerve->setVoltage(newVel);
            }),
            frc2::WaitCommand(runTime),
            frc2::InstantCommand([swerve]{swerve->setVoltage({});}),
            VisionAlign(swerve, visionManager.get(), 1_s),
            ShootNBalls(feeder, 2).WithTimeout(5_s)
            );
}