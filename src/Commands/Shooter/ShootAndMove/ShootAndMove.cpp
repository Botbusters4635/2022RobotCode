//
// Created by abiel on 4/9/22.
//

#include "ShootAndMove.h"
#include "Math/VisionUtilities.h"
#include <frc/MathUtil.h>
#include <iostream>

ShootAndMove::ShootAndMove(const std::shared_ptr<Shooter> &shooter,
                           const std::shared_ptr<EctoSwerve> &swerve,
                           const std::shared_ptr<Feeder> &feeder,
                           const std::shared_ptr<VisionManager> &vision,
                           const std::shared_ptr<PIDTurret> &turret,
                           const ShootAndMoveConfig &config) {
    this->shooter = shooter;
    this->feeder = feeder;
    this->vision = vision;
    this->swerve = swerve;
    this->config = config;
    this->shooter = shooter;
    this->turret = turret;

    frc2::CommandBase::SetName("ShootAndMove");
    AddRequirements({shooter.get(), feeder.get(), turret.get()});
}

void ShootAndMove::Initialize() {
//    initialWait = true;
//    initialTime = frc::Timer::GetFPGATimestamp().value();
    visionAnglePID.Reset(units::radian_t(vision->getYawError()));
    visionAnglePID.SetTolerance(0.15_rad);
    xFilter.Reset();
    yFilter.Reset();
    allowShootTime = frc::Timer::GetFPGATimestamp();
    turretError = turret->getHeadingToRobot().Radians().value();


    table = ntInstance.GetTable("ShootAndMove");
}

void ShootAndMove::Execute() {
//    if (initialWait){
//        if (frc::Timer::GetFPGATimestamp().value() - initialTime  > 0.5){
//            initialWait = false;
//        }else{
//            initialWait = true;
//        }
//    }else{
//
//    }

    frc::SmartDashboard::PutData("ShootAndMovePID", &visionAnglePID);
    auto distanceToTarget = vision->getTargetDistance();
    frc::Pose2d pose = swerve->getPose();
    auto swerveVel = swerve->getVelocity();
    auto yaw = pose.Rotation().Radians().value();
    swerveVel = frc::Velocity2d(
            units::meters_per_second_t(xFilter.Calculate(swerveVel.X().value())),
            units::meters_per_second_t(yFilter.Calculate(swerveVel.Y().value())));
    auto flywheelVel = units::radians_per_second_t(shooter->getVelocity());

    /**
     * Calculate offset to target created by the ball's initial velocity (chassis velocity)
     */
    //Swerve velocity is in local frame, transform to global frame
    auto global_swerveVel = swerveVel.RotateBy(swerve->getRotation());
    auto ballTOF = units::second_t(config.tofTable.get(distanceToTarget));

    auto ballOffset = global_swerveVel * ballTOF.value();
    ballOffset *= 0.977;

    /**
     * Actual target position will change depending on the ball offset,
     * aim for the position with the inverse of the offset
     */
    frc::Translation2d baseTargetPosition = frc::Pose2d(8.25_m, 4.07_m, {0_deg}).Translation();
    frc::Translation2d targetPosition = baseTargetPosition - frc::Translation2d(units::meter_t(ballOffset.X().value()),
                                                                                units::meter_t(ballOffset.Y().value()));

//    frc::Translation2d targetPosition = baseTargetPosition;

    distanceToTarget = pose.Translation().Distance(targetPosition).value(); //Update the new distance

    double errorAngle = std::atan2(
            (targetPosition.Y() - pose.Y()).value(),
            (targetPosition.X() - pose.X()).value());

//    errorAngle -= pose.Rotation().RotateBy(frc::Rotation2d(
//            units::degree_t(180))).Radians().value(); //Might need to be offset by 90 deg or something, depending on cs
    errorAngle -= pose.Rotation().RotateBy(frc::Rotation2d(units::degree_t(0))).Radians().value();
    errorAngle = EctoMath::wrapAngle(errorAngle); //Calculates the new error to center

    turretError = visionFilter.Calculate(errorAngle);
    turretError = movingAvFilter.Calculate(turretError);

    /**
     * Given the new distance to target and angle to target, recalculate flywheel velocities and swerve yaw
     */
    auto targetFlywheelVel = config.shooterTable.get(distanceToTarget);
    auto targetFlywheelHood = config.hoodTable.get(distanceToTarget);

    shooter->setSetpoint(targetFlywheelVel);
    shooter->setHood(targetFlywheelHood);
//    turret->setToRobot(errorAngle * 1_rad);
    turret->setToRobot(units::radian_t(turretError));

    /**
     * If solution converges, shoot shoot
     */
    bool flywheelWithinSetpoint = std::abs(flywheelVel.value() - targetFlywheelVel) < 15;
    bool hoodAtSetpoint = shooter->hoodAtSetpoint();
    bool targetVelWithinRange = swerve->getVelMagitude() < 0.55 or true;
    bool turretWithinSetPoint = std::abs(turret->getHeadingToRobot().Radians().value() - (turretError)) < 0.2;
    bool turretAngularVelWithingRange = std::abs(turret->getVel().value()) < 1;

    bool allowShoot = flywheelWithinSetpoint && hoodAtSetpoint && targetVelWithinRange && turretWithinSetPoint && turretAngularVelWithingRange;
    if (!allowShoot){
        allowShootTime = frc::Timer::GetFPGATimestamp();
        table->GetEntry("allowShootTime").SetBoolean("False");
    } else{
        table->GetEntry("allowShootTime").SetBoolean("True");
    }
    if (allowShoot && frc::Timer::GetFPGATimestamp() - allowShootTime > 300_ms) {
        //Shoot shoot
        feeder->setFeederVol(12);
        if(ballCount != feeder->getBallsShot()){
            std::cout << fmt::format("Shot ball at: {}, offset distance: {}", vision->getTargetDistance(), distanceToTarget) << std::endl;
            ballCount = feeder->getBallsShot();
        }

    } else {
        doPreload();
    }

    /**
     * Telemetry, woop woop
     */
    table->GetEntry("BallTOF").SetDouble(ballTOF.value());
    table->GetEntry("BallOffset/X").SetDouble(ballOffset.X().value());
    table->GetEntry("BallOffset/Y").SetDouble(ballOffset.Y().value());
    table->GetEntry("TargetPosition/X").SetDouble(targetPosition.X().value());
    table->GetEntry("TargetPosition/Y").SetDouble(targetPosition.Y().value());
    table->GetEntry("Convergence/FlywheelDelta").SetDouble(flywheelVel.value() - targetFlywheelVel);
    table->GetEntry("Convergence/YawErrorDelta").SetDouble(errorAngle - vision->getYawError());
    table->GetEntry("ErrorAngle").SetDouble(errorAngle);
    table->GetEntry("Convergence/DistanceErrorDelta").SetDouble(distanceToTarget - vision->getTargetDistance());
    table->GetEntry("Convergence/FlywheelWithinSetpoint").SetBoolean(flywheelWithinSetpoint);
    table->GetEntry("SwerveVel/dX").SetDouble(swerveVel.X().value());
    table->GetEntry("SwerveVel/dY").SetDouble(swerveVel.Y().value());
    table->GetEntry("ShooterHood").SetDouble(shooter->getEstimatedHoodAngle().value());
    VisionUtilities::publishPoint(table->GetEntry("OriginalTargetPosition"), {8.25, 4.07});
    VisionUtilities::publishPoint(table->GetEntry("OffsetedTargetPosition"),
                                  {targetPosition.X().value(), targetPosition.Y().value()});

}

void ShootAndMove::End(bool interrupted) {
    feeder->setFeederVol(0);
}

void ShootAndMove::doPreload() {
    //idk how else to run a command inside a command without messing up requirements, so this
    if (!feeder->getFeederLimit()){
        if(!feeder->getMiddleLimit() && !feeder->getIntakeLimit()){
            feeder->setFeederVol(12);
        }
        if(feeder->getMiddleLimit() && !feeder->getIntakeLimit()){
            feeder->setFeederVol(0);
        }
        if(feeder->getIntakeLimit() && feeder->getMiddleLimit()){
            feeder->setFeederVol(6);
        }
        if (!feeder->getMiddleLimit() && feeder->getIntakeLimit()){
            feeder->setFeederVol(12);
        }
    }else{
        feeder->setFeederVol(0);
    }
}

bool ShootAndMove::IsFinished() {
    return false;
}