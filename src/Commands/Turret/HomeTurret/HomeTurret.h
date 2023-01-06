//
// Created by cc on 19/05/22.
//

#ifndef BOTBUSTERS_REBIRTH_HOMETURRET_H
#define BOTBUSTERS_REBIRTH_HOMETURRET_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/StartEndCommand.h>

#include "Systems/PIDTurret/PIDTurret.h"

class HomeTurret : public frc2::CommandHelper<frc2::SequentialCommandGroup, HomeTurret> {
public:
    HomeTurret(const std::shared_ptr<PIDTurret> &turret);


};


#endif //BOTBUSTERS_REBIRTH_HOMETURRET_H
