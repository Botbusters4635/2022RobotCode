//
// Created by hiram on 28/06/19.
//

#ifndef  BOTBUSTERS_REBIRTH_ECTOROBOT_H
#define BOTBUSTERS_REBIRTH_ECTOROBOT_H

#include <frc2/command/CommandScheduler.h>
#include <frc/TimedRobot.h>
#include <map>
#include <Core/EctoModule/System.h>
#include <Core/EctoModule/SystemHandler.h>
#include <spdlog/spdlog.h>
#include <Core/MotorHandler/MotorManager.h>
#include <Core/EctoInput/InputManager.h>
#include <Core/EctoModule/ManagerHandler.h>

#include "Systems/TimingDataPublisher/TimingDataPublisher.h"
#include "Core/EctoRobotConfig.h"

#include "Core/PCM/PCMManager.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "Core/EctoChecklist/ChecklistRunner.h"

class EctoRobot : public frc::TimedRobot, public System {
public:
	explicit EctoRobot(const std::string &robotName, units::millisecond_t period = 20_ms);

	PCMManager &pcmManager = PCMManager::getInstance();
	MotorManager &motorManager = MotorManager::getInstance();
	InputManager &inputManager = InputManager::getInstance();
	
	virtual void autoInit() { ; };
	
	virtual void autoUpdate() { ; };
	
	virtual void teleopInit() { ; };
	
	virtual void teleopUpdate() { ; };
	
	virtual void testInit() { ; };
	
	virtual void testUpdate() { ; };

    virtual void simInit() {;};

    virtual void simUpdate() {;};

protected:
	/**
	 * set this in order to define which motors are @ which ids
	 * @return
	 */
	virtual std::list<MotorInfo> getMotorConfig() = 0;
	
	virtual std::list<PistonInfo> getPistonConfig() {
		enablePistonManager = false;
		return {};
	}
	
	frc2::CommandScheduler &commandScheduler = frc2::CommandScheduler::GetInstance();

    ManagerHandler &managerHandler = ManagerHandler::getInstance();

//    ChecklistRunner checklistRunner;

	bool hasInit{false};

private:

	bool enablePistonManager = true;
	
	void RobotInit() final;
	
	void RobotPeriodic() final;
	
	void TestInit() final;
	
	void TestPeriodic() final;
	
	void DisabledInit() final;
	
	void DisabledPeriodic() final;
	
	void TeleopInit() final;
	
	void TeleopPeriodic() final;
	
	void AutonomousInit() final;
	
	void AutonomousPeriodic() final;

    void SimulationInit() final;

    void SimulationPeriodic() final;

    bool testsCreated{false};
};


#endif //BOTBUSTERS_REBIRTH_ECTOROBOT_H
