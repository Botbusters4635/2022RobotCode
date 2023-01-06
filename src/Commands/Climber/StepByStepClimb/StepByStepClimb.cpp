//
// Created by abiel on 1/31/22.
//

#include "StepByStepClimb.h"
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include "Commands/Climber/ReleaseClimber/ReleaseClimber.h"

#include "Commands/Climber/HomeClimber/HomeClimber.h"
#include "Commands/Climber/UseClimber/UseClimber.h"
#include "Commands/Climber/ClimberToLimitSwitch/ClimberToLimitSwitch.h"
#include "Commands/Utilities/WaitForButton/WaitForButton.h"

StepByStepClimb::StepByStepClimb(const std::shared_ptr<PIDClimber>& climber) {
    this->AddRequirements(climber.get());



    AddCommands(
            ReleaseClimber(climber, true),

            WaitForButton("A", 1),

            UseClimber(climber, 0.60, PIDClimber::FeedForward::Loaded),

            WaitForButton("A", 1),

            UseClimber(climber, 0, PIDClimber::FeedForward::Loaded),

            WaitForButton("A", 1),

            UseClimber(climber, 0.1, PIDClimber::FeedForward::Loaded),

                    frc2::ConditionalCommand(frc2::PrintCommand("Climber already homed! Skipping"),
                                     HomeClimber(climber),
                                     [climber = climber] { return climber->hasClimberHomed(); }
                                    ),

            frc2::PrintCommand("Ranking point get! (hopefully)")
    );
}