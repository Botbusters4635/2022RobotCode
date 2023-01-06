//
// Created by cc on 31/08/22.
//

#ifndef BOTBUSTERS_REBIRTH_SHOOT_H
#define BOTBUSTERS_REBIRTH_SHOOT_H


#include "Systems/Feeder/Feeder.h"
#include "Systems/PIDTurret/PIDTurret.h"
#include "Core/VisionManager/VisionManager.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>


class Shoot : public frc2::CommandHelper<frc2::CommandBase, Shoot>{
public:
    explicit Shoot(const std::shared_ptr<Feeder> &feeder);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<Feeder> feeder;

    double startTime;
    bool hasRun{false};
};


#endif //BOTBUSTERS_REBIRTH_SHOOT_H
