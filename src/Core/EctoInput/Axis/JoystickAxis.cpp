//
// Created by Abiel on 9/11/18.
//

#include "Core/EctoInput/Axis/JoystickAxis.h"

JoystickAxis::JoystickAxis() : frc2::Button(
        [this]{
            return this->get() > 0.4;
        }
        ) {
    ;
}

JoystickAxis::JoystickAxis(JoystickAxis &&other) noexcept : JoystickAxis() {
    this->outValue = other.outValue;
}

void JoystickAxis::updateValue(double value) {
	std::lock_guard<std::mutex> lock(joystickMutex);
	this->outValue = calculateOutput(value);
}

double JoystickAxis::calculateOutput(double value) {
	return value;
}

double JoystickAxis::get() const {
	std::lock_guard<std::mutex> lock(joystickMutex);
	return outValue;
}



