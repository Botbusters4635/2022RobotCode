#include "SwerveSteerCharacterizationRobot.h"

SwerveSteerCharacterizationRobot::SwerveSteerCharacterizationRobot() : EctoCharacterizationRobot("SwerveSteerCharacterization"){
    
}

void SwerveSteerCharacterizationRobot::robotInit(){
    steerMotor = motorManager.getMotor("front_left_steer");
	steerMotor->setFeedbackMode(MotorFeedbackMode::QuadEncoder);
    steerMotor->prioritizeUpdateRate();
}

void SwerveSteerCharacterizationRobot::teleopUpdate(){

}

void SwerveSteerCharacterizationRobot::robotUpdate(){

}

void SwerveSteerCharacterizationRobot::autoInit(){
    logger.InitLogging();
}

void SwerveSteerCharacterizationRobot::autoUpdate(){
    double pos = steerMotor->getPosition() / 18.0 / (2 * M_PI);
    double vel = steerMotor->getVelocity() / 18.0 / (2 * M_PI);
    frc::SmartDashboard::PutNumber("pos", pos);
    frc::SmartDashboard::PutNumber("vel", vel);

    double voltage = logger.GetMotorVoltage().value();
    steerMotor->set(voltage, MotorControlMode::Voltage);

    logger.Log(pos, vel);
}

void SwerveSteerCharacterizationRobot::disabledInit(){
    steerMotor->set(0);
    logger.SendData();
}