//
// Created by cc on 08/10/22.
//

#ifndef BOTBUSTERS_REBIRTH_WAITTURRETONTARGET_H
#define BOTBUSTERS_REBIRTH_WAITTURRETONTARGET_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Utilities/WPI/SlewRateLimiter/RateLimiter.h"

#include <networktables/NetworkTable.h>

#include "Systems/PIDTurret/PIDTurret.h"
#include "Core/VisionManager/VisionManager.h"

#include <frc/controller/ProfiledPIDController.h>

class WaitTurretOnTarget : public frc2::CommandHelper<frc2::CommandBase, WaitTurretOnTarget>{
public:
    WaitTurretOnTarget(const std::shared_ptr<PIDTurret> &turret);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<PIDTurret> turret;
};

#endif //BOTBUSTERS_REBIRTH_WAITTURRETONTARGET_H
