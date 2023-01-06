//
// Created by abiel on 1/31/22.
//

#ifndef BOTBUSTERS_REBIRTH_STEPBYSTEPCLIMB_H
#define BOTBUSTERS_REBIRTH_STEPBYSTEPCLIMB_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Systems/PIDClimber/PIDClimber.h"

#include "Core/EctoInput/InputManager.h"

class StepByStepClimb : public frc2::CommandHelper<frc2::SequentialCommandGroup, StepByStepClimb> {
public:
    explicit StepByStepClimb(const std::shared_ptr<PIDClimber>& climber);
};


#endif //BOTBUSTERS_REBIRTH_STEPBYSTEPCLIMB_H
