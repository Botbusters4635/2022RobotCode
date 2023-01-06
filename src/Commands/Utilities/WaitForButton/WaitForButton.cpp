//
// Created by abiel on 2/1/22.
//

#include "WaitForButton.h"
#include <iostream>

WaitForButton::WaitForButton(const std::string &buttonName, int joystickId) {
    this->buttonName = buttonName;
    this->joystickId = joystickId;
}

void WaitForButton::Initialize() {
    std::cout << "Waiting for button\n";
    waitingEntry.SetBoolean(true);
    InputManager::getInstance().registerButton(button, buttonName, joystickId);
}

void WaitForButton::End(bool interrupted) {
        std::cout << "Button pressed\n";
        waitingEntry.SetBoolean(false);
}

bool WaitForButton::IsFinished() {
    return button.get();
}


