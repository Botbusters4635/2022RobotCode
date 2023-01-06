//
// Created by abiel on 2/7/22.
//

#ifndef BOTBUSTERS_REBIRTH_SHOOTNBALLS_H
#define BOTBUSTERS_REBIRTH_SHOOTNBALLS_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Feeder/Feeder.h"

class ShootNBalls : public frc2::CommandHelper<frc2::CommandBase, ShootNBalls> {
public:
    ShootNBalls(const std::shared_ptr<Feeder> &feeder, int balls = 2);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<Feeder> feeder;
    units::second_t startTime;
    int initialCount{}, lastDelayCount{};
    units::second_t delayStartTime;
    int nBalls;
    bool hasRun;
};


#endif //BOTBUSTERS_REBIRTH_SHOOTNBALLS_H
