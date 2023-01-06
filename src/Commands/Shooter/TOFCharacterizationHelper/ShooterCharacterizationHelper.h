//
// Created by abiel on 4/6/22.
//

#ifndef BOTBUSTERS_REBIRTH_SHOOTERCHARACTERIZATIONHELPER_H
#define BOTBUSTERS_REBIRTH_SHOOTERCHARACTERIZATIONHELPER_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/Feeder/Feeder.h"
#include "Systems/Intake/Intake.h"
#include "Systems/Generic/Shooter.h"

class ShooterCharacterizationHelper : public frc2::CommandHelper<frc2::SequentialCommandGroup, ShooterCharacterizationHelper> {
    ShooterCharacterizationHelper(const std::shared_ptr<EctoSwerve> &swerve,
                                  const std::shared_ptr<Shooter> &shooter,
                                  const std::shared_ptr<Feeder> &feeder,
                                  const std::shared_ptr<Intake> &intake);
};


#endif //BOTBUSTERS_REBIRTH_SHOOTERCHARACTERIZATIONHELPER_H
