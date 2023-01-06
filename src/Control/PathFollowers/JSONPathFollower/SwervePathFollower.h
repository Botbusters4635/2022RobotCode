//
// Created by abiel on 3/30/22.
//

#ifndef BOTBUSTERS_REBIRTH_SWERVEPATHFOLLOWER_H
#define BOTBUSTERS_REBIRTH_SWERVEPATHFOLLOWER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/controller/ProfiledPIDController.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "SwerveTrajectory.h"

#include "Control/Feedforward/HolonomicFeedforward.h"

#include <frc/smartdashboard/Field2d.h>

struct SwervePathFollowerConfig {
    double pos_p{0}, pos_i{0}, pos_d{0};
    double theta_p{0}, theta_i{0}, theta_d{0};

    HolonomicFeedforward ff;
};

class SwervePathFollower : public frc2::CommandHelper<frc2::CommandBase, SwervePathFollower> {
public:
    SwervePathFollower(const std::shared_ptr<EctoSwerve> &swerve, SwerveTrajectory &&trajectory, const SwervePathFollowerConfig &config);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<EctoSwerve> swerve;
    SwerveTrajectory path;

    SwervePathFollowerConfig config;

    std::unique_ptr<frc2::PIDController> xController, yController, thetaController;

    units::second_t startTime{0_s};
    units::second_t lastTime {0_s};

    units::second_t getTime() const {
        return frc::Timer::GetFPGATimestamp() - startTime;
    }

    HolonomicFeedforward ff;
    frc::Velocity2d lastVelocity;

    void updateTelemetry(const SwerveTrajectory::State &state, const frc::ChassisSpeeds &out, const frc::Vector2d &ff);
    std::shared_ptr<nt::NetworkTable> table;

    frc::Field2d field;
};


#endif //BOTBUSTERS_REBIRTH_SWERVEPATHFOLLOWER_H
