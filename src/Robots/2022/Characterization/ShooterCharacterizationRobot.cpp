//
// Created by cc on 30/01/22.
//

#include "ShooterCharacterizationRobot.h"

ShooterCharacterizationRobot::ShooterCharacterizationRobot() : EctoCharacterizationRobot("SwerveSteerCharacterization"){

}

void ShooterCharacterizationRobot::robotInit(){
    masterMotor = motorManager.getMotor("shooterMotor");
    slaveMotor = motorManager.getMotor("invertedShooterMotor");
    slaveMotor->followMotor(*masterMotor, true);
    slaveMotor->setFeedbackMode(MotorFeedbackMode::QuadEncoder);

    masterMotor->prioritizeUpdateRate();
}

void ShooterCharacterizationRobot::teleopUpdate(){

}

void ShooterCharacterizationRobot::robotUpdate(){

}

void ShooterCharacterizationRobot::autoInit(){
    logger.InitLogging();
}

void ShooterCharacterizationRobot::autoUpdate(){
    double pos = (masterMotor->getPosition() / gearRatio) / (2 * M_PI);
    double vel = (masterMotor->getVelocity() / gearRatio) / (2 * M_PI);
    frc::SmartDashboard::PutNumber("pos", pos);
    frc::SmartDashboard::PutNumber("vel", vel);

    double voltage = logger.GetMotorVoltage().value();
    masterMotor->set(voltage, MotorControlMode::Voltage);

    logger.Log(pos, vel);
}

void ShooterCharacterizationRobot::disabledInit(){
    masterMotor->set(0);
    logger.SendData();
}