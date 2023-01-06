//
// Created by andrew on 10/11/21.
//

#ifndef BOTBUSTERS_REBIRTH_LQRSHOOTER_H
#define BOTBUSTERS_REBIRTH_LQRSHOOTER_H

#include <frc/system/LinearSystemLoop.h>
#include <frc/estimator/KalmanFilter.h>
#include "Systems/Generic/Shooter.h"

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>

#include <networktables/NetworkTableInstance.h>
#include <frc/system/plant/LinearSystemId.h>
#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/StateSpaceUtil.h>
#include <frc/TimedRobot.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <wpi/numbers>

struct ShooterLQRConfig {
    std::vector<std::shared_ptr<EctoMotor>> motors;
    std::vector<bool> isInverted;

    units::volt_t kV;
    units::volt_t kA;

    double gearRatio = 1;

    units::radians_per_second_squared_t maxAcceleration = 300_rad_per_s_sq;
};


class LQRShooter : public Shooter {
public:
    explicit LQRShooter(const ShooterLQRConfig &config);

    void robotInit() override;

    void robotUpdate() override;

    double getVelocity() const override;

    void setSetpoint(double velocity) override;

    bool flywheelAtSetpoint() override;

private:
    ShooterLQRConfig config;
    std::vector<std::shared_ptr<EctoMotor>> motors;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    units::radians_per_second_t targetSetpoint{0_rad_per_s};
    frc::SlewRateLimiter<units::rad_per_s> slewRateLimiter;

    frc::LinearSystem<1,1,1> flywheelPlant;
    frc::KalmanFilter<1,1,1> observer;
    frc::LinearQuadraticRegulator<1,1> controller;
    frc::LinearSystemLoop<1,1,1> loop;
};


#endif //BOTBUSTERS_REBIRTH_LQRSHOOTER_H
