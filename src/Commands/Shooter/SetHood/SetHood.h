//
// Created by cc on 10/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_SETHOOD_H
#define BOTBUSTERS_REBIRTH_SETHOOD_H

#include "Systems/Generic/Shooter.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>


class SetHood :  public frc2::CommandHelper<frc2::CommandBase, SetHood>{
public:
    SetHood(const std::shared_ptr<Shooter> &shooter, double setState);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<Shooter> shooter;
    double setState;
    double startTime{};

};


#endif //BOTBUSTERS_REBIRTH_SETHOOD_H
