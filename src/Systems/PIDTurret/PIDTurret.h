//
// Created by cc on 11/05/22.
//

#ifndef BOTBUSTERS_REBIRTH_PIDTURRET_H
#define BOTBUSTERS_REBIRTH_PIDTURRET_H

#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <Core/EctoModule/WPISubsystem.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalGlitchFilter.h>
#include <frc/controller/ProfiledPIDController.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include "Math/EctoMath.h"


struct PIDTurretConfig {
    PIDConfig pidConfig;
    std::shared_ptr<EctoMotor> turretMotor;

    bool isInverted = false;
    double rampRate = 0.01;
    double currentLimit = 20;
    double gearReduction = 0;

    bool enableForwardSoftLimit = true;
    bool enableReverseSoftLimit = true;

    units::radian_t forwardSoftLimit = 0_rad; //radians
    units::radian_t reverseSoftLimit = 0_rad; //radians

//    double minAngle = 0; //radians
//    double maxAngle = 270; //radians

    units::radian_t turretZeroOffsetToRobot;

    units::meter_t turretToCamera;
    frc::Translation2d turretCenterToRobotCenter;

    MotorControlMode motorControlMode = MotorControlMode::Voltage;
    MotorFeedbackMode motorFeedbackMode = MotorFeedbackMode::QuadEncoder;

    units::radians_per_second_t maxVel = 0_rad_per_s;
    units::radians_per_second_squared_t maxAccel = 0_rad_per_s_sq;

    units::radian_t pidDistTol = 1_rad;
    units::radians_per_second_t  pidVelTol = 1_rad_per_s;

    frc::SimpleMotorFeedforward<units::radians> ff {0.0_V, 0.0_V / 1_rad_per_s, 0.0_V / 1_rad_per_s_sq};



};

class PIDTurret : public WPISubsystem{
public:
    explicit PIDTurret(const PIDTurretConfig &config);

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
    bool hasTurretHomed() const {return hasHomed;}

    void resetController();

    void usePIDControl(bool usePID);

    void useSoftLimits(bool useSoftLimits);

    void resetToZero();

    void setSensorAngle(units::radian_t angle);

    //Makes it so that any angle is transformed to between min and max eg. 0 and 270, -135 becomes 135 etc.
    //Radians are used, degrees are just easier to describe in a sense.

    static double deWrapAngle(double radians) ;

    bool atDeadZone(){return inDeadZone;};

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

    frc::Rotation2d simGetHeadingToRobot(){
        return units::radian_t(EctoMath::wrapAngle(EctoMath::degreesToRadians(simHeading) + config.turretZeroOffsetToRobot.value()));
    }

    frc::Transform2d getCameraToRobot();

    frc::Transform2d simGetCameraToRobot();

private:
//    mutable std::mutex mutex;

    PIDTurretConfig config;
    bool hasHomed = false;
    bool usePID = true;
    double manualVoltage;
    std::shared_ptr<EctoMotor> motor;

    mutable std::mutex turretMutex{};

    std::unique_ptr<frc::ProfiledPIDController<units::radians>> pidController;
    std::unique_ptr<frc::TrapezoidProfile<units::radians>::Constraints> trapezoidProfileConstraints;
    frc::SimpleMotorFeedforward<units::radians>* ff;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    double simHeading;

    bool inDeadZone;





};


#endif //BOTBUSTERS_REBIRTH_PIDTURRET_H
