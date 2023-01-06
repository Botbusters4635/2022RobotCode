//
// Created by abiel on 4/15/22.
//

#ifndef BOTBUSTERS_REBIRTH_TAXICOMMAND_H
#define BOTBUSTERS_REBIRTH_TAXICOMMAND_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Core/VisionManager/VisionManager.h"
#include "Systems/Feeder/Feeder.h"

class TaxiCommand : public frc2::CommandHelper<frc2::SequentialCommandGroup, TaxiCommand> {
public:
    TaxiCommand(const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<VisionManager> &visionManager, const std::shared_ptr<Feeder> &feeder,
                const frc::Pose2d &initialPose,
                const frc::ChassisSpeeds &targetVoltage, units::second_t waitTime, units::second_t runTime);
};


#endif //BOTBUSTERS_REBIRTH_TAXICOMMAND_H
