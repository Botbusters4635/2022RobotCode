//
// Created by abiel on 4/6/22.
//

#ifndef BOTBUSTERS_REBIRTH_MOVETOPOINT_H
#define BOTBUSTERS_REBIRTH_MOVETOPOINT_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Core/VisionManager/VisionManager.h"

struct MoveToPointConfig {
    double pos_p{0}, pos_i{0}, pos_d{0};
    double theta_p{0}, theta_i{0}, theta_d{0};

    units::meters_per_second_t maxVel{1_mps};
    units::meters_per_second_squared_t maxAccel{0.5_mps_sq};

    units::radians_per_second_t maxAngularVel{2_rad_per_s};
    units::radians_per_second_squared_t maxAngularAccel{3_rad_per_s_sq};
};

class MoveToPoint : public frc2::CommandHelper<frc2::CommandBase, MoveToPoint> {
public:
    MoveToPoint(const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<VisionManager> &vision, const frc::Pose2d &targetPose, const MoveToPointConfig &config);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    MoveToPointConfig config;

    std::shared_ptr<EctoSwerve> swerve;
    std::shared_ptr<VisionManager> visionManager;
    frc::Pose2d targetPose;

    frc::ProfiledPIDController<units::meter> xController, yController;
    frc::ProfiledPIDController<units::radians> thetaController;
};


#endif //BOTBUSTERS_REBIRTH_MOVETOPOINT_H
