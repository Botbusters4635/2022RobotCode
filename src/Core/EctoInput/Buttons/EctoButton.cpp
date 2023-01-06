//
// Created by Abiel on 9/11/18.
//

#include "EctoButton.h"
#include <iostream>

EctoButton::EctoButton() : frc2::Button(
        [this]{
            return this->get();
        }
        ) {
    ;
}

EctoButton::EctoButton(EctoButton &&other) : EctoButton()  {
    this->outValue = other.outValue;
}

EctoButton::EctoButton(const EctoButton &other) : EctoButton() {
    this->outValue = other.outValue;
}

void EctoButton::updateStatus(bool status) {
	std::lock_guard<std::mutex> lock(buttonMutex);
	outValue = calculateOutput(status);
}

bool EctoButton::calculateOutput(bool input) {
	return input;
}

bool EctoButton::get() const {
	std::lock_guard<std::mutex> lock(buttonMutex);
	return outValue;
}

#include "Core/EctoInput/InputManager.h"

EctoButton::~EctoButton() {
    std::cout << "Destructor called" << std::endl;
    InputManager::getInstance().unregisterButton(this);
}

