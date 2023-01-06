//
// Created by abiel on 2/26/22.
//

#ifndef BOTBUSTERS_REBIRTH_SWERVEDISTANCEALIGN_H
#define BOTBUSTERS_REBIRTH_SWERVEDISTANCEALIGN_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/controller/PIDController.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include <frc/filter/MedianFilter.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/DutyCycle.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

class SwerveDistanceAlign : public frc2::CommandHelper<frc2::CommandBase, SwerveDistanceAlign> {
public:
    SwerveDistanceAlign(const std::shared_ptr<EctoSwerve> &swerve, units::meter_t distance, units::radian_t targetHeading);

    SwerveDistanceAlign(const std::shared_ptr<EctoSwerve> &swerve, units::meter_t distance);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End(bool interrupted) override;
private:
    units::meter_t getDistances();
    bool validPulse{false};

    std::shared_ptr<EctoSwerve> swerve;
    frc::Rotation2d targetRot;
    units::meter_t targetDistance;
    bool doHeadingAlign{false};

    frc::ProfiledPIDController<units::meters> distanceController{6.60,0.31,0.98, {3.4_mps, 6_mps_sq}};
    frc::SlewRateLimiter<units::meters> distanceSlewLimiter{6_mps};

    frc::ProfiledPIDController<units::radian> rotationController{3.81, 0.21, 0.0, {5_rad_per_s, 10_rad_per_s_sq}};

    frc::AnalogInput distanceSensor{3};
};


#endif //BOTBUSTERS_REBIRTH_SWERVEDISTANCEALIGN_H
