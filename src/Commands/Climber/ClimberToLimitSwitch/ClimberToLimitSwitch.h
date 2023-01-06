//
// Created by abiel on 2/1/22.
//

#ifndef BOTBUSTERS_REBIRTH_CLIMBERTOLIMITSWITCH_H
#define BOTBUSTERS_REBIRTH_CLIMBERTOLIMITSWITCH_H

#include "Systems/PIDClimber/PIDClimber.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ClimberToLimitSwitch : public frc2::CommandHelper<frc2::CommandBase, ClimberToLimitSwitch> {
public:
    ClimberToLimitSwitch(const std::shared_ptr<PIDClimber> &climber, double setpoint = -12.0);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
private:
    std::shared_ptr<PIDClimber> climber;
    double setpoint;
};


#endif //BOTBUSTERS_REBIRTH_CLIMBERTOLIMITSWITCH_H
