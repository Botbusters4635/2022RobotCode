////
//// Created by cc on 3/11/22.
////
//
//#ifndef BOTBUSTERS_REBIRTH_PIDJOINT_H
//#define BOTBUSTERS_REBIRTH_PIDJOINT_H
//
//#include "Core/EctoModule/WPISubsystem.h"
//#include <frc/controller/PIDController.h>
//#include <frc/controller/ArmFeedforward.h>
//#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
//#include <units/current.h>
//#include <units/angle.h>
//#include <units/angular_acceleration.h>
//#include <units/angular_velocity.h>
//#include <networktables/NetworkTableInstance.h>
//#include <networktables/NetworkTable.h>
//#include <frc/controller/ProfiledPIDController.h>
//
//struct PIDJointConfig{
//    PIDConfig pidConfig;
//    std::vector<std::shared_ptr<EctoMotor>> motors;
//
//    std::vector<bool> isInverted;
//
//    double rampRate = 0.01;
//    double gearReaduction = 1;// in/out
//
//    units::ampere_t currentLimit = 20_A;
//
//    bool enableForwardSoftLimit = true;
//    bool enableReverseSoftLimit = true;
//
//    units::radian_t forwardSoftLimit = 0_rad;
//    units::radian_t reverseSoftLimit = 0_rad;
//
//    units::radian_t zOffset = 0_rad;
//
//    units::radians_per_second_t maxVel = 0_rad_per_s;
//    units::radians_per_second_squared_t maxAccel = 0_rad_per_s_sq;
//
//    units::radian_t pidDistTol = 1_rad;
//    units::radians_per_second_t pidVelTol = 1_rad_per_s;
//
//    frc::ArmFeedforward ff{0.0_V, 0.0_V, 0.0_V / 1_rad_per_s, 0.0_V / 1_rad_per_s_sq};
//
//};
//
//
//class PIDJoint : public WPISubsystem{
//public:
//    PIDJoint(const PIDJointConfig &config);
//
//    void robotInit() override;
//    void robotUpdate() override;
//
//    bool getForwardLimitSwitch();
//    bool getReverseLimitSwitch();
//
//    units::radian_t getAngle();
//
//    units::radians_per_second_t getVel();
//
//    void set(units::radian_t radians);
//
//    void set(double radians){
//        set(units::radian_t(radians));
//    }
//
//    bool getHome() {return hasHomed;}
//
//    void resetController();
//
//    void usePIDControl(bool usePID);
//
//    void useSoftLimits(bool useSoftLimits);
//
//    void setSensorAngle(units::radian_t angle);
//
//    void setCurrent(units::ampere_t amps);
//
//    bool atGoal();
//
//    void setHomed(bool home){
//        hasHomed = home;
//    }
//
//    void setVoltage(double set){manualVoltage = set;}
//
//    void setPID(PIDConfig &config);
//
//    units::volt_t calculateFF(units::radians_per_second_t vel);
//
//
//private:
//
//    std::vector<std::shared_ptr<EctoMotor>> motors;
//    PIDConfig pidConfig;
//    PIDJointConfig config;
//
//    std::unique_ptr<frc::ProfiledPIDController<units::radians>> pidController;
//    std::unique_ptr<frc::TrapezoidProfile<units::radians>::Constraints> constraints;
//
//    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
//    std::shared_ptr<nt::NetworkTable> table;
//
//    bool hasHomed{false};
//    double manualVoltage;
//};
//
//
//#endif //BOTBUSTERS_REBIRTH_PIDJOINT_H
