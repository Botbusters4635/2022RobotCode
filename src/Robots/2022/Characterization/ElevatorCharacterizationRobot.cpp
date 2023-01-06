#include "ElevatorCharacterizationRobot.h"


ElevatorCharacterizationRobot::ElevatorCharacterizationRobot() : EctoCharacterizationRobot("ElevatorCharacterization"){
    
}

void ElevatorCharacterizationRobot::robotInit(){
    elevator = motorManager.getMotor("elevator");
    elevator1 = motorManager.getMotor("elevatorFollower");
    elevator2 = motorManager.getMotor("elevatorFollowerSecond");

	elevator->setFeedbackMode(MotorFeedbackMode::QuadEncoder);
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


    inputManager.registerAxis(climberAxis, "rightY");

//    elevator->prioritizeUpdateRate();
    gearBox = pcm.getPiston("gearBox");
    PCMManager::set(gearBox, false);
}

void ElevatorCharacterizationRobot::teleopUpdate(){
//    if(elevator->getReverseLimitSwitch()){
//        elevator->setSensorPosition(0.0);
//    }
    elevator->set(-climberAxis.get(), MotorControlMode::Percent);

}

void ElevatorCharacterizationRobot::robotUpdate(){
    frc::SmartDashboard::PutNumber("Climber Position", elevator->getPosition());

}

void ElevatorCharacterizationRobot::autoInit(){
    logger.InitLogging();
}

void ElevatorCharacterizationRobot::autoUpdate(){
// you need to send the raw rotationes to sysId and
// in sysId you have to set the units per rotation.
    double pos = (elevator->getPosition() / gearRatioElevator / (2 * M_PI));
    double vel = (elevator->getVelocity() / gearRatioElevator / (2 * M_PI));

    frc::SmartDashboard::PutNumber("pos", pos);
    frc::SmartDashboard::PutNumber("vel", vel);

    double voltage = logger.GetMotorVoltage().value();
    elevator->set(voltage, MotorControlMode::Voltage);

    logger.Log(pos, vel);
}

void ElevatorCharacterizationRobot::disabledInit(){
    logger.SendData();
    elevator->set(0);
}