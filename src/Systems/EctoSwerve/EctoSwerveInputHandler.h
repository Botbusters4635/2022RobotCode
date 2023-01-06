//
// Created by abiel on 1/2/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H
#define BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H

#include "frc2/command/CommandBase.h"
#include "frc2/command/CommandHelper.h"

#include "EctoSwerve.h"
#include "Core/EctoModule/System.h"
#include <Core/EctoInput/InputManager.h>
#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/Buttons/ToggleButton.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/filter/SlewRateLimiter.h>
#include <frc/controller/PIDController.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include "Core/VisionManager/VisionManager.h"
#include "Systems/PIDShooter/PIDShooter.h"

#include <frc/filter/MedianFilter.h>

class EctoSwerveInputHandler : public frc2::CommandHelper<frc2::CommandBase, EctoSwerveInputHandler> {
public:
	explicit EctoSwerveInputHandler(const std::shared_ptr<EctoSwerve> &swerveIn,
                                    VisionManager *visionManagerIn);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

    void setVisionYawOffset(double offset);

private:
    std::shared_ptr<EctoSwerve> swerve;
    VisionManager * visionManager;
    InputManager& input = InputManager::getInstance();


	/**
	 * NT
	 */
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table, visionTable, aprilTagTable;
	nt::NetworkTableEntry visionAngle, hasTarget, aprilTagYaw, aprilTagX;

	const double joystickExpo = 0.25;
	const double joystickDeadzone = 0.15;
	
	const double triggerExpo = 0.2;
	const double triggerDeadzone = 0.2;
	
	JoystickAxisExpo strafeAxis{joystickExpo, joystickDeadzone}, forwardAxis{joystickExpo,
	                                                                         joystickDeadzone}, rotationAxis{
			0.2, 0.2};

    JoystickAxisExpo fieldOrientedAxis{triggerExpo, triggerDeadzone}, shootWhileMove{0.2, 0.2};

	EctoButton resetYaw, visionButton, disableFieldOriented, brake, shootAndMoveButton, alignX, alignY, alignYaw;
	units::meters_per_second_t maximumInputAcceleration{60.4}; // / 1s
	units::radians_per_second_t maximumInputAngularAcceleration{M_PI * 8.25}; // / 1S
	frc::SlewRateLimiter<units::meters_per_second> xFilter{maximumInputAcceleration / units::second_t(1.0)};
	frc::SlewRateLimiter<units::meters_per_second> yFilter{maximumInputAcceleration / units::second_t(1.0)};
	frc::SlewRateLimiter<units::radians_per_second> thetaFilter{maximumInputAngularAcceleration / units::second_t(1.0)};

//    frc::ProfiledPIDController<units::radian> visionAnglePID{0.35, 0.0, 0.0012, {10_rad_per_s, 20_rad_per_s_sq}};
    frc::PIDController yawPID{0.5, 0, 0};
    frc::PIDController xPID{0.5,0,0};
    double visionOffset{0};

	static bool registeredJoysticks;
    bool lastVisionAlign {false};
    double orbitModeDirection{1};
    double newSetPoint;
    bool enableRobotOriented{false};

    frc::SlewRateLimiter<units::radian> yawOffsetLimiter{units::radian_t(0.15*M_PI) / units::second_t(1)};
};


#endif //BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H
