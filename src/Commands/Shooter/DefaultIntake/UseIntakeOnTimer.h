//
// Created by cc on 05/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_USEINTAKEONTIMER_H
#define BOTBUSTERS_REBIRTH_USEINTAKEONTIMER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Intake/Intake.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include "Core/EctoInput/InputManager.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Timer.h>


class UseIntakeOnTimer : public frc2::CommandHelper<frc2::CommandBase, UseIntakeOnTimer>{
public:
    explicit UseIntakeOnTimer(const std::shared_ptr<Intake> &intake, double target, bool state, double time);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<Intake> intake;
    double time;
    bool state;
    double target;
    double startTime{};


};


#endif //BOTBUSTERS_REBIRTH_USEINTAKEONTIMER_H
