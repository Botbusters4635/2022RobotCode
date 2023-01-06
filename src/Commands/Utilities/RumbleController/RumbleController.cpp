//
// Created by cc on 24/02/22.
//

#include "RumbleController.h"

RumbleController::RumbleController(double leftRumble, double rightRumble, int controllerID, double time) {
    this->leftRumble = leftRumble;
    this->rightRumble = rightRumble;
    this->controllerID = controllerID;
    this->time = time;
}

void RumbleController::Initialize() {
    startTime = frc::Timer::GetFPGATimestamp().value();
}
void RumbleController::Execute() {
    input.setControllerRumble(leftRumble, rightRumble, controllerID);
}
void RumbleController::End(bool interrupted) {
    input.setControllerRumble(leftRumble, rightRumble, controllerID);
}
bool RumbleController::IsFinished() {
    if ((frc::Timer::GetFPGATimestamp().value() - startTime) > time){
        input.setControllerRumble(0,0, controllerID);
        return true;
    }else{
        return false;
    }
}