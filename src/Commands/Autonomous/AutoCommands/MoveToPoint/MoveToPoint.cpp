//
// Created by abiel on 4/6/22.
//

#include "MoveToPoint.h"

MoveToPoint::MoveToPoint(const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<VisionManager> &vision, const frc::Pose2d &targetPose, const MoveToPointConfig &config) :
        xController(config.pos_p, config.pos_i, config.pos_d, {config.maxVel, config.maxAccel}),
        yController(config.pos_p, config.pos_i, config.pos_d, {config.maxVel, config.maxAccel}),
        thetaController(config.theta_p, config.theta_i, config.theta_d, {config.maxAngularVel, config.maxAngularAccel})
    {
    this->config = config;
    this->swerve = swerve;
    this->visionManager = vision;
    AddRequirements(swerve.get());

    this->targetPose = targetPose;
}

void MoveToPoint::Initialize() {
    auto pose = swerve->getPose();
    xController.Reset(pose.X());
    yController.Reset(pose.Y());
    thetaController.Reset(pose.Rotation().Radians());

    thetaController.EnableContinuousInput(-3.14_rad, 3.14_rad);
}

void MoveToPoint::Execute() {
    auto pose = swerve->getPose();
    frc::ChassisSpeeds out;
    out.vx = units::meters_per_second_t(xController.Calculate(pose.X(), targetPose.X()));
    out.vy = units::meters_per_second_t(yController.Calculate(pose.Y(), targetPose.Y()));

    units::meter_t distToTarget = pose.Translation().Distance(targetPose.Translation());
    if(distToTarget < 0.5_m){
        out.omega = -units::radians_per_second_t (thetaController.Calculate(targetPose.Rotation().Radians(), 0_rad));
    }

    swerve->setVoltage(out);
}

void MoveToPoint::End(bool interrupted) {
    swerve->setVoltage({});
}

bool MoveToPoint::IsFinished() {
    auto pose = swerve->getPose();
    auto dist = pose.Translation().Distance(targetPose.Translation());
    auto rot = std::abs(pose.Rotation().Radians().value() - targetPose.Rotation().Radians().value());
    return dist < 0.1_m && rot < 0.1;
}