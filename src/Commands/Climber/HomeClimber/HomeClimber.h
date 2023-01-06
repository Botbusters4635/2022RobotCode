//
// Created by cc on 28/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_HOMECLIMBER_H
#define BOTBUSTERS_REBIRTH_HOMECLIMBER_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Systems/PIDClimber/PIDClimber.h"

/**
 * Mimics 3D printer style home, does two homes, one at high vel, then one at slow vel
 */
class HomeClimber : public frc2::CommandHelper<frc2::SequentialCommandGroup, HomeClimber>{
public:
    explicit HomeClimber(const std::shared_ptr<PIDClimber> &climber);
};


#endif //BOTBUSTERS_REBIRTH_HOMECLIMBER_H
