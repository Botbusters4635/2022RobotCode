//
// Created by Neil Rodriguez Murillo on 10/8/21.
//

#ifndef BOTBUSTERS_REBIRTH_INTAKEINPUTHANDLER_H
#define BOTBUSTERS_REBIRTH_INTAKEINPUTHANDLER_H

#include <Core/EctoInput/InputManager.h>
#include "Systems/Intake/Intake.h"

#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>


class IntakeInputHandler : public frc2::CommandHelper<frc2::CommandBase, IntakeInputHandler> {
public:
	explicit IntakeInputHandler(const std::shared_ptr<Intake> &intake) {
		this->intake = intake;
	}

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
	//CONTROL INPUTS left and right shoulder button are switched
	EctoButton leftShoulder;
	JoystickAxisExpo xRightJoy{0.1, 0.45};
	EctoButton rightShoulder;
	//CONTROL INPUT NAMES
	static constexpr auto intakeJoy = "leftY";
	static constexpr auto intakePistonButton = "rightBumper";
	static constexpr auto intakePistonAndSpitButton = "leftBumper";
	
	
	//INSTANCES
	InputManager &input = InputManager::getInstance();
	std::shared_ptr<Intake> intake;
	IntakeConfig config;
	
};


#endif //BOTBUSTERS_REBIRTH_INTAKEINPUTHANDLER_H
