//
// Created by abiel on 1/16/22.
//

#include "ElevatorTesting.h"

ElevatorTesting::ElevatorTesting() : EctoRobot("ElevatorTesting") {
    ;
}



void ElevatorTesting::teleopInit() {
    elevator = motorManager.getMotor("elevator");
    elevator1 = motorManager.getMotor("elevatorFollower");
    elevator2 = motorManager.getMotor("elevatorFollowerSecond");



    double currentLimit = 20;
    elevator->enableBrakingOnIdle(true);
    elevator->setMotorCurrentLimit(currentLimit);
    elevator->enableCurrentLimit(true);
    elevator->invertMotor(true);
    elevator->prioritizeUpdateRate();
    elevator->burnFlash();


    elevator1->followMotor(*elevator, false);
    elevator1->prioritizeUpdateRate();
    elevator1->enableBrakingOnIdle(true);
    elevator1->setMotorCurrentLimit(currentLimit);
    elevator1->enableCurrentLimit(true);
    elevator1->burnFlash();

    elevator2->followMotor(*elevator, false);
    elevator2->prioritizeUpdateRate();
    elevator2->enableBrakingOnIdle(true);
    elevator2->setMotorCurrentLimit(currentLimit);
    elevator2->enableCurrentLimit(true);
    elevator2->burnFlash();


    inputManager.registerAxis(frontAxis, "rightY");
    inputManager.registerAxis(backAxis, "leftY");
    inputManager.registerButton(gearBoxButton, "leftBumper");
    frc::SmartDashboard::PutBoolean("gearBox", false);
}

double ElevatorTesting::radToHeight(double rads) {
    double motorRotations = rads / (2.0 * M_PI);
    motorRotations /= 5.25252525;
    double height = motorRotations * (0.1516);
    return height;
}

void ElevatorTesting::updateTelemetry(){
    frc::SmartDashboard::PutNumber("pose", elevator->getPosition());
    frc::SmartDashboard::PutNumber("realPose", radToHeight(elevator->getPosition()));
    frc::SmartDashboard::PutNumber("joystickOut", frontAxis.get());


}
void ElevatorTesting::teleopUpdate() {
    updateTelemetry();
    elevator->set((-frontAxis.get()), MotorControlMode::Percent);
//    PCMManager::set(gearBox, frc::SmartDashboard::GetBoolean("gearBox", false));

}