//
// Created by cc on 08/06/22.
//

#include "TurretCharacterizationRobot.h"

TurretCharacterizationRobot::TurretCharacterizationRobot() : EctoCharacterizationRobot("TurretCharacterization"){
    ;
}

void TurretCharacterizationRobot::robotInit() {
    turretMotor = motorManager.getMotor("turretMotor");
    turretMotor->setFeedbackMode(MotorFeedbackMode::QuadEncoder);
    turretMotor->prioritizeUpdateRate();
}

void TurretCharacterizationRobot::teleopUpdate() {;}

void TurretCharacterizationRobot::robotUpdate() {
    frc::SmartDashboard::PutNumber("raw/turretPose", turretMotor->getPosition());
    frc::SmartDashboard::PutNumber("raw/turretVel", turretMotor->getVelocity());
    frc::SmartDashboard::PutNumber("gearRatio/turretPose", turretMotor->getPosition() / 51.388889);
    frc::SmartDashboard::PutNumber("gearRatio/turretVel", turretMotor->getVelocity() / 51.388889);


}

void TurretCharacterizationRobot::autoInit() {
    logger.InitLogging();
}

void TurretCharacterizationRobot::autoUpdate() {
    double pos = turretMotor->getPosition() / 51.388889 / (2 * M_PI);
    double vel = turretMotor->getVelocity() / 51.388889 / (2 * M_PI);
    frc::SmartDashboard::PutNumber("pos", pos);
    frc::SmartDashboard::PutNumber("vel", vel);

    double voltage = logger.GetMotorVoltage().value();
    turretMotor->set(voltage, MotorControlMode::Voltage);

    logger.Log(pos, vel);


}

void TurretCharacterizationRobot::disabledInit() {
    turretMotor->set(0);
    logger.SendData();
}

