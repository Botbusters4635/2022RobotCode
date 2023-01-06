//
// Created by abiel on 3/30/22.
//

#include "SwervePathFollower.h"

SwervePathFollower::SwervePathFollower(const std::shared_ptr<EctoSwerve> &swerve, SwerveTrajectory &&trajectory, const SwervePathFollowerConfig &config) :
    path(trajectory) {
    this->swerve = swerve;
    this->config = config;
    this->ff = config.ff;

    AddRequirements(swerve.get());

    xController = std::make_unique<frc2::PIDController>(
            config.pos_p, config.pos_i, config.pos_d
            );

    yController = std::make_unique<frc2::PIDController>(
            config.pos_p, config.pos_i, config.pos_d
    );

    thetaController = std::make_unique<frc2::PIDController>(
            config.theta_p, config.theta_i, config.theta_d
    );

    thetaController->EnableContinuousInput(-M_PI, M_PI);
}

void SwervePathFollower::Initialize() {
    table = nt::NetworkTableInstance::GetDefault().GetTable("SwervePathFollower");
    startTime = frc::Timer::GetFPGATimestamp();

    xController->Reset();
    yController->Reset();
    thetaController->Reset();
    field.GetObject("Trajectory")->SetTrajectory(path.getWPITrajectory());
}

void SwervePathFollower::Execute() {
    auto time = getTime();
    auto dt = getTime() - lastTime;

    auto target = path.sample(time);

    auto accel = (target.targetVelocity - lastVelocity) / dt.value();
    auto ffVec = ff.calculateFF({target.targetVelocity.X().value(), target.targetVelocity.Y().value()}, {accel.X().value(), accel.Y().value()});

    auto pose = swerve->getPose();
    auto xOut = xController->Calculate(pose.X().value(), target.targetPose.X().value());
    auto yOut = yController->Calculate(pose.Y().value(), target.targetPose.Y().value());
    auto rotOut = thetaController->Calculate(pose.Rotation().Radians().value(), target.targetPose.Rotation().Radians().value());

    frc::ChassisSpeeds vel;
    vel.vx = units::meters_per_second_t(xOut + ffVec.x);
    vel.vy = units::meters_per_second_t(yOut + ffVec.y);
    vel.omega = units::radians_per_second_t(rotOut);

    vel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(vel.vx, vel.vy, vel.omega,
                                                            pose.Rotation());

    swerve->setVoltage(vel);

    field.SetRobotPose(pose);

    updateTelemetry(target, vel, ffVec);
    lastTime = getTime();
    lastVelocity = target.targetVelocity;
}

void SwervePathFollower::updateTelemetry(const SwerveTrajectory::State &state, const frc::ChassisSpeeds &out, const frc::Vector2d &ff) {
    table->GetEntry("State/Pose/X").SetDouble(state.targetPose.X().value());
    table->GetEntry("State/Pose/Y").SetDouble(state.targetPose.Y().value());

    table->GetEntry("State/Velocity/vX").SetDouble(state.targetVelocity.X().value());
    table->GetEntry("State/Velocity/vY").SetDouble(state.targetVelocity.Y().value());

    table->GetEntry("FF/x").SetDouble(ff.x);
    table->GetEntry("FF/y").SetDouble(ff.y);

    table->GetEntry("ChassisSpeeds/vX").SetDouble(out.vx.value());
    table->GetEntry("ChassisSpeeds/vY").SetDouble(out.vy.value());
    table->GetEntry("ChassisSpeeds/omega").SetDouble(out.omega.value());
    table->GetEntry("TotalTime").SetDouble(path.getTotalTime().value());

    frc::SmartDashboard::PutData("SwervePathFollower", &field);
}

void SwervePathFollower::End(bool interrupted) {
    swerve->setVelocity(frc::ChassisSpeeds());
}

bool SwervePathFollower::IsFinished(){
    return false;
    return getTime() >= path.getTotalTime();
}