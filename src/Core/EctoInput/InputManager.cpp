#include <iostream>
#include <utility>
#include "InputManager.h"
#include <Core/EctoInput/DefaultMappings.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

InputManager::InputManager() : Manager("InputManager") {
	log->info("Initializing EctoInput...");

}

void InputManager::init() {
    addMapping(EctoInput::DefaultMappings::Xbox(), 0);
    addMapping(EctoInput::DefaultMappings::Xbox(), 1);
    addMapping(EctoInput::DefaultMappings::Xbox(), 2);
}

void InputManager::update() {
	if (frc::DriverStation::IsNewControlData()) {
		for (const auto &joystickDataPair: joysticksData) {
			const JoystickData &joyData = joystickDataPair.second;
			const std::shared_ptr<frc::Joystick> &joy = joyData.joystick;
            if (!joy->IsConnected()){continue;}
			auto &mapping = joystickMappings[joy->GetPort()];
			
			for (const auto &registeredButtonPair: joyData.registeredButtons) {
				for (const auto &registeredButton: registeredButtonPair.second) {
					const bool buttonState = joy->GetRawButton(mapping.buttonsMapping[registeredButtonPair.first]);
					//log->info("Getting button: {}", mapping.buttonsMapping[registeredButtonPair.first]);
					registeredButton->updateStatus(buttonState);
				}
			}
			
			for (const auto &registeredAxesPair: joyData.registeredAxes) {
				for (const auto &registeredAxis: registeredAxesPair.second) {
					const double axisValue = joy->GetRawAxis(mapping.axesMapping[registeredAxesPair.first]);
					registeredAxis->updateValue(axisValue);
				}
			}
            for (const auto &registeredDPadButtonPair: joyData.registeredDPadButtons){
                for (const auto &registeredDPadButton: registeredDPadButtonPair.second) {
                    const bool dPadButtonOut = mapping.dPadMapping[registeredDPadButtonPair.first] == joy->GetPOV();
                    registeredDPadButton->updateStatus(dPadButtonOut);
                }
            }

		}
	}
	
}

void InputManager::registerButton(EctoButton &button, const std::string &buttonName, int joystickId) {
	if (joystickMappings[joystickId].buttonsMapping.count(buttonName) == 0) {
		log->warn("Button {} is not defined for Joystick id: {}! Button not registered", buttonName, joystickId);
		return;
	}
	
	if (joysticksData.count(joystickId) == 0) {
		JoystickData newJoystickData;
		newJoystickData.joystick = std::make_shared<frc::Joystick>(joystickId);
		
		newJoystickData.registeredButtons[buttonName].push_back(&button);
		joysticksData[joystickId] = newJoystickData;
	} else {
		auto &registeredButtons = joysticksData[joystickId].registeredButtons;
		auto &registeredButtonsToName = registeredButtons[buttonName];
		auto buttonIterator = std::find(registeredButtonsToName.begin(), registeredButtonsToName.end(), &button);
		
		if (buttonIterator != registeredButtonsToName.end()) {
			log->warn("Tried to register an EctoButton that was already registered! Name: {} on Joystick id: {}",
			          buttonName, joystickId);
			return;
		}
		registeredButtonsToName.push_back(&button);
	}
}

void InputManager::unregisterButton(EctoButton *button) {
    for(auto &joyData : joysticksData){
        auto &data(joyData.second);
        for(auto &buttonPair : data.registeredButtons){
            buttonPair.second.erase(
                    std::remove_if(buttonPair.second.begin(),
                                   buttonPair.second.end(),
                                   [button](EctoButton* b){
                                       return button == b;
                                   }),
                                   buttonPair.second.end()
                    );
        }
    }
}

void InputManager::registerAxis(JoystickAxis &axis, const std::string &axisName, int joystickId) {
	if (joystickMappings[joystickId].axesMapping.count(axisName) == 0) {
		log->warn("Axis {} is not defined for Joystick id: {}! Axis not registered", axisName, joystickId);
		return;
	}
	
	if (joysticksData.count(joystickId) == 0) {
		JoystickData newJoystickData;
		newJoystickData.joystick = std::make_shared<frc::Joystick>(joystickId);
		
		newJoystickData.registeredAxes[axisName].push_back(&axis);
		joysticksData[joystickId] = newJoystickData;
	} else {
		auto &registeredAxes = joysticksData[joystickId].registeredAxes;
		auto &registeredAxesToName = registeredAxes[axisName];
		auto axisIterator = std::find(registeredAxesToName.begin(), registeredAxesToName.end(), &axis);
		
		if (axisIterator != registeredAxesToName.end()) {
			log->warn("Tried to register a JoystickAxis that was already registered! Name: {} on Joystick id: {}",
			          axisName, joystickId);
			return;
		}
		registeredAxesToName.push_back(&axis);
	}
}

void InputManager::setControllerRumble(double leftRumble, double rightRumble, int joystickId) {
	if (joysticksData.count(joystickId) == 0) {
		log->warn("Tried to set Rumble on non registered Joystick id: {}", joystickId);
		return;
	}
	auto &joystick = joysticksData[joystickId].joystick;
	
	joystick->SetRumble(frc::Joystick::RumbleType::kLeftRumble, leftRumble);
	joystick->SetRumble(frc::Joystick::RumbleType::kRightRumble, rightRumble);
}

int InputManager::getPOVAngleReading(int joystickId) {
	if (joysticksData.count(joystickId) == 0) {
		log->warn("Tried to get POV Angle on non registered Joystick id: {}", joystickId);
		return 0;
	}
	return joysticksData[joystickId].joystick->GetPOV();
}

void InputManager::registerDPadButton(EctoButton &button, const std::string &dPadButtonName, int joystickId) {
    if (joysticksData.count(joystickId) == 0) {
        JoystickData newJoystickData;
        newJoystickData.joystick = std::make_shared<frc::Joystick>(joystickId);

        newJoystickData.registeredButtons[dPadButtonName].push_back(&button);
        joysticksData[joystickId] = newJoystickData;
    }else{
        auto &registeredDPadButtons = joysticksData[joystickId].registeredDPadButtons;
        auto &registeredDPadButtonsToName = registeredDPadButtons[dPadButtonName];
        auto dPadButtonIterator = std::find(registeredDPadButtonsToName.begin(), registeredDPadButtonsToName.end(), &button);

        if (dPadButtonIterator != registeredDPadButtonsToName.end()) {
            log->warn("Tried to register a JoystickAxis that was already registered! Name: {} on Joystick id: {}",
                      dPadButtonName, joystickId);
            return;
        }
        registeredDPadButtonsToName.push_back(&button);

//        bool dPadOut = joystickMappings[joystickId].dPadMapping[dPadButtonName] == joysticksData[joystickId].joystick->GetPOV();
//        button.updateStatus(dPadOut);
    }
 }

void InputManager::addMapping(InputManager::JoystickMap mapping, int joystickId) {
	if (joystickMappings.count(joystickId) != 0) {
		log->warn("Overriding previous mapping for joystick id: {}!", joystickId);
	}
	joystickMappings[joystickId] = std::move(mapping);
}

