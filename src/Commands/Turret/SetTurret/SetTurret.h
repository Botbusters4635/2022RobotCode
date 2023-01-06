//
// Created by cc on 19/05/22.
//

#ifndef BOTBUSTERS_REBIRTH_SETTURRET_H
#define BOTBUSTERS_REBIRTH_SETTURRET_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Systems/PIDTurret/PIDTurret.h"

class SetTurret : public frc2::CommandHelper<frc2::CommandBase, SetTurret> {
public:
    SetTurret(const std::shared_ptr<PIDTurret> &turret, double set);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<PIDTurret> turret;
    double set = 0;

};


#endif //BOTBUSTERS_REBIRTH_SETTURRET_H
