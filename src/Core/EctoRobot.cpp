//
// Created by hiram on 28/06/19.
//

#include "EctoRobot.h"
#include <frc/livewindow/LiveWindow.h>
#include "Core/EctoChecklist/ChecklistItem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <memory>

EctoRobot::EctoRobot(const std::string &robotName, units::millisecond_t period) : TimedRobot(period),
                                                     System(robotName) {
    managerHandler.addManager(inputManager);
    managerHandler.addManager(pcmManager);
    managerHandler.addManager(motorManager);

}

void EctoRobot::RobotInit() {
	log->info("RobotInit...");
    frc::DataLogManager::Start();
	frc::LiveWindow::DisableAllTelemetry();
	
	/**
	 * Motor handler initialization
 	*/
	motorManager.setMotorInfo(getMotorConfig());

	/**
	 * PCMManager initialization
	 */
	auto pistonConfig = getPistonConfig();
	if (enablePistonManager) {
		pcmManager.setPCMConfig(pistonConfig);
	}

    frc2::CommandScheduler::GetInstance().OnCommandInitialize(
            [](const frc2::Command& command) {
                frc::Shuffleboard::AddEventMarker(
                        "Command Initialized", command.GetName(),
                        frc::ShuffleboardEventImportance::kNormal);
            });
    frc2::CommandScheduler::GetInstance().OnCommandInterrupt(
            [](const frc2::Command& command) {
                frc::Shuffleboard::AddEventMarker(
                        "Command Interrupted", command.GetName(),
                        frc::ShuffleboardEventImportance::kNormal);
            });
    frc2::CommandScheduler::GetInstance().OnCommandFinish(
            [](const frc2::Command& command) {
                frc::Shuffleboard::AddEventMarker(
                        "Command Finished", command.GetName(),
                        frc::ShuffleboardEventImportance::kNormal);
            });

    managerHandler.init();
    robotInit();
	hasInit = true;
}


void EctoRobot::RobotPeriodic() {
    managerHandler.update();
    commandScheduler.Run();
    robotUpdate();
}

//TODO Run command handler here
void EctoRobot::AutonomousInit() {
	commandScheduler.CancelAll();
	log->info("AutonomousInit... ");
	autoInit();
}

void EctoRobot::AutonomousPeriodic() {
	frc::SmartDashboard::PutData(&commandScheduler);
	autoUpdate();
}

void EctoRobot::TeleopInit() {
	commandScheduler.CancelAll();
	log->info("TeleopInit...");
	teleopInit();
}

void EctoRobot::TeleopPeriodic() {
	teleopUpdate();
}

void EctoRobot::DisabledInit() {
	log->info("DisabledInit...");
    commandScheduler.Enable();
    this->disabledInit();
}

void EctoRobot::DisabledPeriodic() {
	this->disabledUpdate();
}

//TODO Add test runner
void EctoRobot::TestInit() {
    log->info("TestInit...");
	commandScheduler.Disable();
	commandScheduler.CancelAll();

    if(!testsCreated){
        //auto testItem = std::make_shared<ChecklistTest>();
        //checklistRunner.addItems("ChecklistTest", {testItem});

//        checklistRunner.addItems(motorManager.getName(), motorManager.createTests());
//
//        checklistRunner.robotInit();
    }

	this->testInit();
}

void EctoRobot::TestPeriodic() {
//	checklistRunner.robotUpdate();
	this->testUpdate();
}

void EctoRobot::SimulationInit() {
	if(!hasInit) throw std::runtime_error("RobotInit not run");
    simInit();
}

void EctoRobot::SimulationPeriodic() {
    simUpdate();
}