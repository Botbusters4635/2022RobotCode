//
// Created by cc on 24/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_RUMBLECONTROLLER_H
#define BOTBUSTERS_REBIRTH_RUMBLECONTROLLER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Core/EctoInput/InputManager.h"
#include "Systems/Feeder/Feeder.h"
#include <frc/Timer.h>

class RumbleController : public frc2::CommandHelper<frc2::CommandBase, RumbleController>{
public:
    explicit RumbleController(double leftRumble, double rightRumble, int controllerID, double time);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    InputManager& input = InputManager::getInstance();
    double leftRumble, rightRumble;
    int controllerID;
    double time;
    double startTime{};
};


#endif //BOTBUSTERS_REBIRTH_RUMBLECONTROLLER_H
