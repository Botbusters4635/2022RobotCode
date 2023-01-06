//
// Created by cc on 05/09/22.
//

#ifndef BOTBUSTERS_REBIRTH_TURRETINPUTHANDLER_H
#define BOTBUSTERS_REBIRTH_TURRETINPUTHANDLER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/PIDTurret/PIDTurret.h"
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include "Core/EctoInput/InputManager.h"

class TurretInputHandler : public frc2::CommandHelper<frc2::CommandBase, TurretInputHandler>{
public:
    explicit TurretInputHandler(const std::shared_ptr<PIDTurret> &turret);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<PIDTurret> turret;


    InputManager& input = InputManager::getInstance();

    static bool registeredJoysticks;



    JoystickAxisExpo turretAxis{0.2, 0.2};
    EctoButton lock, manualTurret, endCommand;



};
#endif //BOTBUSTERS_REBIRTH_TURRETINPUTHANDLER_H
