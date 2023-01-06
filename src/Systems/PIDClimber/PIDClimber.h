//
// Created by cc on 29/01/22.
//


#ifndef BOTBUSTERS_REBIRTH_PIDCLIMBER_H
#define BOTBUSTERS_REBIRTH_PIDCLIMBER_H

#include <Core/MotorHandler/MotorManager.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include "Utilities/NetworkTablePID/NetworkTablePID.h"
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/DigitalInput.h>
#include <Core/PCM/PCMManager.h>
#include "Core/EctoModule/WPISubsystem.h"


struct PIDClimberConfig {
    double currentLimit = 20;

    double maxVelocity = 1;
    double maxAcceleration = 0.5;

    double rampRate = 0.1;

    std::vector<std::shared_ptr<EctoMotor>> motors;
    std::vector<bool> isInverted;

    std::shared_ptr<frc::DoubleSolenoid> hookPistons;
    std::shared_ptr<frc::DoubleSolenoid> climberPistons;

    double gearReduction = 1; //Motor Rotations / Rotations Out
    double pulleyDiameter = 1;

    frc::ElevatorFeedforward<units::meters> freeFF {0.0_V, 0.0_V, 0.0_V / 1_mps, 0.0_V / 1_mps_sq};
    frc::ElevatorFeedforward<units::meters> loadedFF {0.0_V, 0.0_V, 0.0_V / 1_mps, 0.0_V / 1_mps_sq};

    PIDConfig pidConfig;
//    PIDConfig springPID;
//    PIDConfig springLessPID;
    double pidTolerance = 0.1;

    bool enableLimitSwitch = true;

    double reverseSoftLimit = 0;
    double forwardSoftLimit = 1;
};

class PIDClimber : public WPISubsystem {
public:
    enum class FeedForward{
        Loaded,
        Free
    };

    explicit PIDClimber(const PIDClimberConfig &config);

    void robotInit() override;

    void robotUpdate() override;

    void setPID(const PIDConfig &pidConfig);

    units::meter_t getHeight() const;

    units::meters_per_second_t getVelocity() const;

    void set(units::meter_t height);

    void set(double height){
        set(units::meter_t(height));
    }

    void setVoltage(double setVoltage);

    bool getLimitSwitch() const;

    void usePIDControl(bool usePID);

    void resetToZero();

    bool isAtGoal();

    void setFF(FeedForward ff);

    void useSoftLimits(bool set);

    bool hasClimberHomed() const {
        return hasHomed;
    }

    void setClimberHomed(bool home) {
        hasHomed = home;
    }

private:
    void resetController();

    double radToHeight(double rads) const;

    double heightToRad(double height) const;

    PIDClimberConfig config{};
    std::vector<std::shared_ptr<EctoMotor>> motors;

    PCMManager &pcm = PCMManager::getInstance();

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;
    nt::NetworkTableEntry masterMotorCurrent, slaveMotorCurrent;

    std::unique_ptr<frc::ProfiledPIDController<units::meters>> pidController;
    frc::ElevatorFeedforward<units::meters>* ff;

    bool usePID{true};

    double manualVoltage{};

    bool hasHomed{false};

//    PIDConfig springPID;
//    PIDConfig springLessPID;

};


#endif //BOTBUSTERS_REBIRTH_PIDCLIMBER_H

