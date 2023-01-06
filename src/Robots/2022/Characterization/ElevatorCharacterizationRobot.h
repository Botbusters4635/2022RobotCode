//
// Created by abiel on 11/3/21.
//

#ifndef BOTBUSTERS_REBIRTH_ELEVATORCHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_ELEVATORCHARACTERIZATIONROBOT_H

#include "Core/EctoCharacterizationRobot.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include <frc/DoubleSolenoid.h>

#include <sysid/logging/SysIdGeneralMechanismLogger.h>

class ElevatorCharacterizationRobot : public EctoCharacterizationRobot {
public:
	ElevatorCharacterizationRobot();
	
	void robotInit() override;

	void robotUpdate() override;
	
	void teleopUpdate() override;
	
	void autoInit() override;
	
	void autoUpdate() override;
	
	void disabledInit() override;


protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "elevator", 12},
                {EctoMotorType::SparkMax, "elevatorFollower", 11},
                {EctoMotorType::SparkMax, "elevatorFollowerSecond", 10}
        };
    }
    std::list<PistonInfo> getPistonConfig(){
        return {
                {"gearBox", 20, frc::PneumaticsModuleType::REVPH, 0, 1}
        };
    }
private:
    PCMManager &pcm = PCMManager::getInstance();
    std::shared_ptr<EctoMotor> elevator, elevator1, elevator2;
    double pulleyCircumference = 0.95 * 0.0254;//give it the
    // pulleyDiameter and it converts to the circumference
    double gearRatioElevator = 5.2525252525;
//    double gearRatioClimber = 13.236363636;
	
	sysid::SysIdGeneralMechanismLogger logger;
    JoystickAxisExpo climberAxis {0.2, 0.2};

    std::shared_ptr<frc::DoubleSolenoid> gearBox;

};


#endif //BOTBUSTERS_REBIRTH_ELEVATORCHARACTERIZATIONROBOT_H
