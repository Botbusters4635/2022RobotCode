//
// Created by cc on 12/06/22.
//

#ifndef BOTBUSTERS_REBIRTH_LQRTURRET_H
#define BOTBUSTERS_REBIRTH_LQRTURRET_H

#include <frc/system/LinearSystemLoop.h>
#include <frc/estimator/KalmanFilter.h>
#include "Systems/Generic/Shooter.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <Core/EctoModule/WPISubsystem.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalGlitchFilter.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include "Math/EctoMath.h"
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

struct LQRTurretConfig {
    std::shared_ptr<EctoMotor> turretMotor;

    bool isInverted = false;
    double rampRate = 0.01;
    double currentLimit = 20;
    double gearReduction = 0;

    units::volt_t kV;
    units::volt_t kA;

    units::volt_t maxControlVoltage = 12_V;

    bool enableForwardSoftLimit = true;
    bool enableReverseSoftLimit = true;

    units::radian_t forwardSoftLimit = 0_rad; //radians
    units::radian_t reverseSoftLimit = 0_rad; //radians

    double minAngle = 0; //radians
    double maxAngle = 270; //radians

    units::radian_t turretZeroOffsetToRobot;

    units::meter_t turretToCamera;
    frc::Translation2d turretCenterToRobotCenter;

    MotorControlMode motorControlMode = MotorControlMode::Voltage;
    MotorFeedbackMode motorFeedbackMode = MotorFeedbackMode::QuadEncoder;

    units::radians_per_second_t maxVel = 0_rad_per_s;
    units::radians_per_second_squared_t maxAccel = 0_rad_per_s_sq;

    units::radian_t distTol = 1_rad;
    units::radians_per_second_t  velTol = 1_rad_per_s;

};

class LQRTurret : public WPISubsystem{
public:
    explicit LQRTurret(const LQRTurretConfig &config);

    void robotInit() override;
    void robotUpdate() override;

    bool getForwardLimitSwitch();
    bool getReverseLimitSwitch();

    units::radian_t getHeading();

    units::radians_per_second_t getVel();

    void set(units::radian_t radians);

    void set(double radians){
        set(units::radian_t(radians));
    }

    void setToRobot(units::radian_t radians);

//    void setToRobot(double radians){
//        setToRobot(units::radian_t(radians));
//    }
    bool hasTurretHomed() {return hasHomed;}

    void resetController();

    void usePIDControl(bool usePID);

    void useSoftLimits(bool useSoftLimits);

    void resetToZero();

    void setSensorAngle(units::radian_t angle);

    //Makes it so that any angle is transformed to between min and max eg. 0 and 270, -135 becomes 135 etc.
    //Radians are used, degrees are just easier to describe in a sense.

    static double deWrapAngle(double radians) ;

    bool atDeadZone(){return inDeadZone;};

    void updateTelemetry();

    void setCurrentLimit(double amps){motor->setMotorCurrentLimit(amps);};

    void setConfigCurrentLimit(){motor->setMotorCurrentLimit(config.currentLimit);};

    bool atGoal();

    void setTurretHomed(bool home){
        hasHomed = home;
    }

    void setVoltage(double set){manualVoltage = set;};

    void setPID(PIDConfig &pidConfig);

    units::volt_t calculateFF(units::radians_per_second_t vel);

    frc::Rotation2d getHeadingToRobot() {
        return units::radian_t(EctoMath::wrapAngle(getHeading().value() + config.turretZeroOffsetToRobot.value()));
    }


    frc::Transform2d getCameraToRobot();

    frc::Transform2d simGetCameraToRobot();


private:
    LQRTurretConfig config;

    bool useLQR;
    bool hasHomed;
    double manualVoltage;
    std::shared_ptr<EctoMotor> motor;

    units::radian_t targetSetpoint{0_rad};

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    frc::SlewRateLimiter<units::rad> velLimiter;
    frc::SlewRateLimiter<units::rad_per_s> accelLimiter;

    frc::LinearSystem<2,1,1> turretPlant;
    frc::KalmanFilter<2,1,1> observer;
    frc::LinearQuadraticRegulator<2,1> controller;
    frc::LinearSystemLoop<2,1,1> loop;

    frc::TrapezoidProfile<units::radians>::State lastProfiledReference;
    frc::TrapezoidProfile<units::radians>::Constraints constraints;


    bool inDeadZone;

};


#endif //BOTBUSTERS_REBIRTH_LQRTURRET_H
