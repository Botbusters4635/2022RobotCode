#pragma once

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include <frc/Notifier.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#include "Control/EctoPID/PIDConfig.h"

#include <frc/geometry/Rotation2d.h>
#include <units/velocity.h>
#include <units/time.h>

#include "Control/Kinematics/Swerve/SwerveWheel.h"
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/LinearFilter.h>
#include <frc/controller/SimpleMotorFeedforward.h>

struct SwerveModuleConfig {
	PIDConfig steerPID, wheelPID;
	
	double analogOffset = 0;
	double wheelCurrentLimit = 25;
	double steerCurrentLimit = 20;
	
	bool enableAnalogFilter = false;
	
	double wheelCircumference = 0;
	double gearRatio = 0;

    frc::SimpleMotorFeedforward<units::radian> steerFF {0_V, 0_V / 1_rad_per_s, 0_V / 1_rad_per_s_sq};
	frc::SimpleMotorFeedforward<units::meter> wheelFF {0_V, 0_V / 1_mps, 0_V / 1_mps_sq};

	frc::TrapezoidProfile<units::radian>::Constraints steerConstraints {0_rad_per_s, 0_rad_per_s_sq};
};

class SwerveModule {
public:
	SwerveModule(const std::string &name, const std::shared_ptr<EctoMotor> &steerMotor,
	             const std::shared_ptr<EctoMotor> &wheelMotor, const SwerveModuleConfig &config);
	
	SwerveWheel getState() const;
	
	void setState(const frc::Rotation2d &rot, units::meters_per_second_t vel) {
		return setState(rot.Radians().value(), vel.value(), MotorControlMode::Velocity);
	}
	
	void setState(double angle, double motorSetpoint, MotorControlMode wheelControlMode);
	
	void setSteerPID(const PIDConfig &config) {
		steerController->SetPID(config.p, config.i, config.d);
	}
	
	void setWheelPID(const PIDConfig &config) {
		wheelMotor->setPIDConfig(config);
	}

private:
	std::string moduleName;
	
	void updateNT();
	
	std::shared_ptr<nt::NetworkTable> table;
	
	static constexpr units::millisecond_t ntUpdateRate{100};
	
	std::shared_ptr<EctoMotor> steerMotor, wheelMotor;
	
	std::unique_ptr<frc2::PIDController> steerController;
	std::unique_ptr<frc2::PIDController> wheelController;
	
	std::unique_ptr<frc::Notifier> ntNotifier, pidNotifier;

    mutable std::mutex stateMutex;
	
	std::unique_ptr<frc::LinearFilter<double>> analogFilter;
	
	MotorControlMode controlMode{MotorControlMode::Percent};
	double wheelSetpoint{0}, steerSetpoint{0};
	
	double gearRatio, wheelCircumference;
	double analogOffset;
	bool lastDsEnabled = false;
    nt::NetworkTableEntry ffEntry;
	nt::NetworkTableEntry velocityEntry, angleEntry, wheelSetpointEntry, steerSetpointEntry, controlModeEntry, steerCurrentEntry, wheelCurrentEntry;
	nt::NetworkTableEntry encoderState;
	nt::NetworkTableEntry rawAnalog;

    mutable double lastWheelPosition{0};
    mutable units::second_t lastStateUpdate{0_s};

    const bool usePosDeltaVelocity = false;
};