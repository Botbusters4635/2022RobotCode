//
// Created by 4ndre on 04/10/2021.
//

#ifndef BOTBUSTERS_REBIRTH_PIDSHOOTER_H
#define BOTBUSTERS_REBIRTH_PIDSHOOTER_H


#include "Systems/Generic/Shooter.h"
#include <frc/controller/ProfiledPIDController.h>
#include <units/length.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <Control/EctoPID/PIDConfig.h>
#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
#include <Core/MotorHandler/MotorManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "Utilities/NetworkTablePID/NetworkTablePID.h"
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Servo.h>

struct PIDShooterConfig {
    PIDConfig pidConfig;

    units::radians_per_second_squared_t maxAcceleration{250};

    double rampRate = 0.1;
    double currentLimit = 30;

    std::vector<std::shared_ptr<EctoMotor>> motors;

    double gearReduction = 1;

    std::vector<bool> isInverted;

    frc::SimpleMotorFeedforward<units::radian> ff {0.0_V, 0.0_V / 1_rad_per_s, 0.0_V / 1_rad_per_s_sq};

    std::vector<int> hoodServoIds = {0, 1};
    /***
     * The distance (meters) from the hood axle to the rotating base of the piston
     * */
    double distanceFromAxleToServoBase = 0.0;

    /**
     * The distance (meters) from the hood axle to the rotation mount of the piston with the hood
     * */ 
    double distanceFromAxleToHoodLink = 0.0;
    
    double minServoDistance = 0.0;
    double maxServoDistance = 0.5;
};

class PIDShooter : public Shooter {
public:

    explicit PIDShooter(const PIDShooterConfig &PIDconfig);

    void robotInit() override;

    void robotUpdate() override;

    void setPIDConfig(const PIDConfig &pidConfig);

    double getVelocity() const override;

    void setSetpoint(double velocity) override;

    bool flywheelAtSetpoint() override;

private:
    PIDShooterConfig config;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    std::unique_ptr<NetworkTablePID> ntPIDConfig;

    std::vector<std::shared_ptr<EctoMotor>> motors;

    frc2::PIDController pidController {0, 0, 0};
    units::radians_per_second_t setpoint{0_rad_per_s};
    frc::SlewRateLimiter<units::rad_per_s> slewRateLimiter;

    frc::SimpleMotorFeedforward<units::radian> ff {0.059963_V, 0.019764_V / 1_rad_per_s, 0.0018242_V / 1_rad_per_s_sq};
};


#endif //BOTBUSTERS_REBIRTH_PIDSHOOTER_H
