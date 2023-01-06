//
// Created by cc on 05/09/22.
//

#ifndef BOTBUSTERS_REBIRTH_SETCLIMBER_H
#define BOTBUSTERS_REBIRTH_SETCLIMBER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/PIDClimber/PIDClimber.h"
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include "Core/EctoInput/InputManager.h"

class ClimberInputHandler : public frc2::CommandHelper<frc2::CommandBase, ClimberInputHandler>{
public:
    explicit ClimberInputHandler(const std::shared_ptr<PIDClimber> &climber);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    InputManager& input = InputManager::getInstance();

    bool registeredJoysticks;

    JoystickAxisExpo climberAxis{0.2, 0.2};
    EctoButton releaseClimber;

    std::shared_ptr<PIDClimber> climber;


};
#endif //BOTBUSTERS_REBIRTH_SETCLIMBER_H
