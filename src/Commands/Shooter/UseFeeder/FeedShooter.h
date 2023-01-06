//
// Created by cc on 25/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_FEEDSHOOTER_H
#define BOTBUSTERS_REBIRTH_FEEDSHOOTER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Feeder/Feeder.h"

class FeedShooter : public frc2::CommandHelper<frc2::CommandBase, FeedShooter>{
public:
    explicit FeedShooter(const std::shared_ptr<Feeder> &feeder, double targetVoltage, bool endInstanly = true);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<Feeder> feeder;

    bool endInstantly;

    double targetVoltage;
};


#endif //BOTBUSTERS_REBIRTH_FEEDSHOOTER_H
