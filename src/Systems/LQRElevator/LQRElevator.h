////
//// Created by cc on 23/05/22.
////
//
//#ifndef BOTBUSTERS_REBIRTH_LQRELEVATOR_H
//#define BOTBUSTERS_REBIRTH_LQRELEVATOR_H
//
//#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
//#include "Core/EctoModule/WPISubsystem.h"
//
//#include <frc/system/plant/LinearSystemId.h>
//#include <frc/trajectory/TrapezoidProfile.h>
//#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
//#include <frc/controller/LinearPlantInversionFeedforward.h>
//#include <frc/controller/LinearQuadraticRegulator.h>
//#include <frc/estimator/KalmanFilter.h>
//#include <frc/controller/LinearPlantInversionFeedforward.h>
//#include <frc/StateSpaceUtil.h>
//#include <frc/TimedRobot.h>
//#include <frc/controller/LinearPlantInversionFeedforward.h>
//#include <frc/controller/LinearQuadraticRegulator.h>
//#include <frc/drive/DifferentialDrive.h>
//#include <frc/estimator/KalmanFilter.h>
//#include <frc/system/LinearSystemLoop.h>
//#include <frc/system/plant/DCMotor.h>
//#include <frc/system/plant/LinearSystemId.h>
//#include <wpi/numbers>
//#include <networktables/NetworkTable.h>
//#include <networktables/NetworkTableInstance.h>
//
//#include <units/units.h>
//#include <frc/filter/SlewRateLimiter.h>
//#include <frc/DoubleSolenoid.h>
//#include "Systems/GearBox/GearBox.h"
//#include <units/power.h>
//
//struct LQRElevatorConfig {
//    units::volt_t kV;
//    units::volt_t kA;
//    units::volt_t maxControllVoltage = 12_V;
//
//    std::vector<std::shared_ptr<EctoMotor>> motors;
//    std::vector<bool> isInverted;
//
//    units::ampere_t currentLimit = 20_A;
//    double rampRate = 0.01;
//
//    double gearRatio = 1; //motor out / mechanism out
//    units::meter_t pulleyDiameter = 0_m;
//
//    bool enableForwardSoftLimit = true;
//    bool enableReverseSoftLimit = true;
//
//    units::meter_t forwardSoftLimit = 0_m;
//    units::meter_t reverseSoftLimit = 0_m;
//
//    units::meters_per_second_t maxVel = 10_mps;
//    units::meters_per_second_squared_t maxAccel = 10_mps_sq;
//
//    units::meter_t distTol = 1_m;
//    units::meters_per_second_t velTol = 1_mps;
//
//};
//
//class LQRElevator : public WPISubsystem{
//public:
//    enum class FeedForward{
//        Loaded,
//        Free
//    };
//
//    explicit LQRElevator(const LQRElevatorConfig &config, const std::shared_ptr<GearBox> &gearBox);
//
//    void robotInit() override;
//    void robotUpdate() override;
//
//    bool getForwardLimitSwitch();
//    bool getReverseLimitSwitch();
//
//    units::meter_t getHeight();
//    units::meters_per_second_t getVel();
//
//    void set(units::meter_t height);
//
//    void set(double height){
//        set(units::meter_t(height));
//    }
//
//    void setPID(const PIDConfig &pidConfig);
//
//    units::meter_t getHeight() const;
//
//    units::meters_per_second_t getVel() const;
//
//    void setVoltage(units::volt_t voltage);
//
//    void setVoltage(double voltage){
//        setVoltage(units::volt_t(voltage));
//    };
//
//    bool getLimitSwitch() const;
//
//    void setFF(FeedForward ff);
//
//    void usePIDControl(bool usePID);
//
//    void resetToZero();
//
//    bool atGoal();
//
//    void useSoftLimits(bool set);
//
//    bool hasElevatorHomed() const{
//        return hasHomed;
//    }
//
//    void setElevatorHomed(bool home){
//        hasHomed = home;
//    }
//
//    void setSensorAngle(units::meter_t height);
//
//    void updateTelemtry();
//
//private:
//    LQRElevatorConfig config;
//
//    std::vector<std::shared_ptr<EctoMotor>> motors;
//
//    std::shared_ptr<GearBox> gearBox;
//
//    void resetController();
//
//    [[nodiscard]] units::meter_t radToHeight(units::radian_t rads) const;
//
//    [[nodiscard]] units::radian_t heightToRad(units::meter_t height) const;
//
//    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
//    std::shared_ptr<nt::NetworkTable> table;
//    nt::NetworkTableEntry masterMotorCurrent, slaveMotorCurrent;
//
//    bool usePID{true};
//
//    units::meter_t targetSetpoint{0_m};
//
//    frc::SlewRateLimiter<units::meters_per_second> slewRateLimiter;
//
//    frc::LinearSystem<2,1,1> elevatorPlant;
//    frc::KalmanFilter<2,1,1> observer;
//    frc::LinearQuadraticRegulator<2,1> controller;
//    frc::LinearSystemLoop<2,1,1> loop;
//
//    frc::TrapezoidProfile<units::meters>::State lastProfiledReference{};
//    frc::TrapezoidProfile<units::meters>::Constraints constraints{};
//
//    double manualVoltage{};
//
//    bool hasHomed = false;
//};
//
//
//#endif //BOTBUSTERS_REBIRTH_LQRELEVATOR_H
