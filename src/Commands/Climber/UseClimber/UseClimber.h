//
// Created by cc on 29/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_USECLIMBER_H
#define BOTBUSTERS_REBIRTH_USECLIMBER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/PIDClimber/PIDClimber.h"

class UseClimber : public frc2::CommandHelper<frc2::CommandBase, UseClimber>{
public:
    explicit UseClimber(const std::shared_ptr<PIDClimber> &climber, double setHeight, PIDClimber::FeedForward feedForward);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<PIDClimber> climber;
    double setHeight;

    PIDClimber::FeedForward feedForward;


};


#endif //BOTBUSTERS_REBIRTH_USECLIMBER_H
