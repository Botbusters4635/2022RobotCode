//
// Created by abiel on 11/3/21.
//

#pragma once

#include "Core/EctoCharacterizationRobot.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include "Systems/EctoSwerve/EctoSwerve.h"

#include <sysid/logging/SysIdGeneralMechanismLogger.h>

class SwerveSteerCharacterizationRobot : public EctoCharacterizationRobot {
public:
	SwerveSteerCharacterizationRobot();
	
	void robotInit() override;

	void robotUpdate() override;
	
	void teleopUpdate() override;
	
	void autoInit() override;
	
	void autoUpdate() override;
	
	void disabledInit() override;


protected:
	std::list<MotorInfo> getMotorConfig() override {
		return {
		        {EctoMotorType::SparkMax, "front_left_steer",  8},
		        {EctoMotorType::SparkMax, "front_right_steer", 5},
		        {EctoMotorType::SparkMax, "back_left_steer",   2},
		        {EctoMotorType::SparkMax, "back_right_steer",  4},
		};
	};
	
	std::shared_ptr<EctoMotor> steerMotor;
	
	sysid::SysIdGeneralMechanismLogger logger;
};