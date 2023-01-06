//
// Created by abiel on 4/9/22.
//

#ifndef BOTBUSTERS_REBIRTH_DELAYEDBALLSHOOT_H
#define BOTBUSTERS_REBIRTH_DELAYEDBALLSHOOT_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Systems/Feeder/Feeder.h"

/**
 * Adds a (fixed) delay in between ball shots
 */
class DelayedBallShoot : public frc2::CommandHelper<frc2::SequentialCommandGroup, DelayedBallShoot> {
public:
    DelayedBallShoot(const std::shared_ptr<Feeder> &feeder, units::second_t delay = 250_ms);
};


#endif //BOTBUSTERS_REBIRTH_DELAYEDBALLSHOOT_H
