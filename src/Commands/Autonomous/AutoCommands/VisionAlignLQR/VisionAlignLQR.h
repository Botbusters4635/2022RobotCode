//
// Created by abiel on 4/12/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONALIGNLQR_H
#define BOTBUSTERS_REBIRTH_VISIONALIGNLQR_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Core/VisionManager/VisionManager.h"
#include "Control/LQR/SwerveRotationLQR.h"
#include <frc/Timer.h>

class VisionAlignLQR : public frc2::CommandHelper<frc2::CommandBase, VisionAlignLQR> {
public:
    VisionAlignLQR(const std::shared_ptr<EctoSwerve> &swerve, VisionManager *visionManager, units::second_t waitTime = 500_ms);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<SwerveThetaController> thetaController;

    VisionManager *visionManager;
    std::unique_ptr<SwerveRotationLQR> rotationLqr;
    frc::Timer atSetpointTimer;
    units::second_t waitTime;
};


#endif //BOTBUSTERS_REBIRTH_VISIONALIGNLQR_H
