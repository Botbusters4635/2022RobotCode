//
// Created by cc on 09/06/22.
//

#include "TurretTesting.h"
#include <Commands/Turret/TurretVisionAlign/TurretVisionAlign.h>
#include <frc2/command/PerpetualCommand.h>

TurretTesting::TurretTesting() : EctoRobot("TurretTesting") {
    table = ntInstance.GetTable("TurretRobot");
    visionTable = ntInstance.GetTable("limelight");
}

void TurretTesting::disabledInit() {
    ;
}

void TurretTesting::disabledUpdate() {
    ;
}

void TurretTesting::robotInit() {
    table->GetEntry("turret/setTurretPose").SetDouble(0);

//    adis = std::make_unique<frc::ADIS16470_IMU>(
//            frc::ADIS16470_IMU::IMUAxis::kZ, frc::SPI::Port::kOnboardCS0,
//            frc::ADIS16470_IMU::CalibrationTime::_16s);
//    // adis.Calibrate();
//
//    adis->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);

    PIDTurretConfig turretConfig;
    turretConfig.pidConfig.p = 27;
    turretConfig.pidConfig.i = 0;
    turretConfig.pidConfig.d = 1.0779;
    turretConfig.ff = {0.6685_V, 0.94547_V / 1_rad_per_s, 0.051475_V / 1_rad_per_s_sq};
    turretConfig.turretMotor = motorManager.getMotor("turretMotor");
    turretConfig.isInverted = false;
    turretConfig.gearReduction = 51.388889;
    turretConfig.reverseSoftLimit = 0_deg;//0
    turretConfig.forwardSoftLimit = 300_deg;//257
    turretConfig.maxVel = 54_rad_per_s;
    turretConfig.maxAccel = 27_rad_per_s_sq;
    turretConfig.pidVelTol = 0.05_rad_per_s;
    turretConfig.pidDistTol = 0.01_rad;
    turretConfig.enableForwardSoftLimit = true;
    turretConfig.enableReverseSoftLimit = true;
    turretConfig.currentLimit = 20;
    turretConfig.rampRate = 0.01;
    turretConfig.turretToCamera = 8.33_in;
    turretConfig.turretZeroOffsetToRobot = -250_deg;
    turretConfig.turretCenterToRobotCenter = {-3.7_in, 0.142_in};


    turret = std::make_shared<PIDTurret>(turretConfig);

    input.registerAxis(turretX, "leftX", 2);
    input.registerAxis(turretY, "leftY", 2);
    input.registerButton(homeTurret, "A", 2);
    input.registerButton(up, "rightBumper", 2);
    input.registerDPadButton(up, "up", 2);
    input.registerDPadButton(down, "down", 2);
    input.registerDPadButton(left, "left", 2);
    input.registerDPadButton(right, "right", 2);
    turret->resetToZero();



}
void TurretTesting::robotUpdate() {
//    yaw = adis->GetAngle().value() * (M_PI / 180.0);
//    yaw = EctoMath::wrapAngle(yaw);
//    frc::SmartDashboard::PutNumber("yawFromWrap", EctoMath::radiansToDegrees(yaw));
//    frc::SmartDashboard::PutNumber("yawFromDeWrap", EctoMath::radiansToDegrees(turret->deWrapAngle(yaw)));



}
void TurretTesting::autoInit() {
    commandScheduler.Schedule(new HomeTurret(turret));
}

void TurretTesting::autoUpdate() {;}

void TurretTesting::teleopInit() {
    turret->useSoftLimits(true);
//    turret->usePIDControl(false);
//    turretMotor = motorManager.getMotor("turretMotor");
//    double currentLimit = 10;
//    turretMotor->setMotorCurrentLimit(currentLimit);
//    turretMotor->enableCurrentLimit(true);
//    up.WhileHeld(TurretVisionAlign(turret).Perpetually());
//    homeTurret.WhenPressed(HomeTurret(turret));

    frc::Pose2d testOne{{0.5_m, 0.5_m}, 0_deg};
    frc::Pose2d testTwo{{0.1_m, 0.1_m}, 0_deg};
    frc::Transform2d testOutput{testOne, testTwo};
    frc::SmartDashboard::PutNumber("testingTransform/X", testOutput.X().value());
    frc::SmartDashboard::PutNumber("testingTransform/Y", testOutput.Y().value());
    frc::SmartDashboard::PutNumber("testingTransform/Theta", testOutput.Rotation().Degrees().value());


}

void TurretTesting::teleopUpdate() {
//    auto heading = std::atan2(-turretX.get(), -turretY.get());
//    double turretError = frc::SmartDashboard::GetNumber("headingTest", EctoMath::radiansToDegrees(heading));
    double turretPose = table->GetNumber("turret/setTurretPose", 0);



//   double turretError = EctoMath::degreesToRadians(visionTable->GetNumber("tx", 0));
//    bool hasTarget = visionTable->GetNumber("tv", 0) != 0;
//    if (up.get() && hasTarget){
//        double setPoint = 0;
//        double state = (turretError);
//        frc::SmartDashboard::PutNumber("state", EctoMath::radiansToDegrees(state));
//        frc::SmartDashboard::PutNumber("setPoint", EctoMath::radiansToDegrees(setPoint));
//        double visionOut = visionPID.Calculate(state, setPoint);
//        turret->set(turret->getHeading().value() + visionOut);
//        frc::SmartDashboard::PutNumber("visionOut", EctoMath::radiansToDegrees(visionOut));
//    }


//    turret->set(yaw);
//    frc::SmartDashboard::PutNumber("yawTest", yaw);
//    frc::SmartDashboard::PutNumber("yawTest", yaw);
//    turret->deWrapAngle(yaw);


//    if(down.get()){
//        turret->set(0);
//    }

//    if (left.get()){
//        adis->Reset();
//    }

    frc::SmartDashboard::PutNumber("turretTesting/CamPose/X", turret->getCameraToRobot().X().value());
    frc::SmartDashboard::PutNumber("turretTesting/CamPose/Y", turret->getCameraToRobot().Y().value());
    frc::SmartDashboard::PutNumber("turretTesting/CamPose/Heading", turret->getCameraToRobot().Rotation().Degrees().value());

    frc::SmartDashboard::PutNumber("turret/getHeading", turret->getHeading().value());
    frc::SmartDashboard::PutNumber("turret/getHeadingToRobot", turret->getHeadingToRobot().Degrees().value());

//    if (!isnan(heading)){
//        turret->set(turretError);
//    }

//    turret->set(turret->getHeading().value() + turretX.get());
//    frc::SmartDashboard::PutNumber("testing/tx", turretError);
    turret->set(EctoMath::degreesToRadians(turretPose));

//    turret->setVoltage(turretPose);
//    turretMotor->set(turretPose, MotorControlMode::Percent);



//    down.WhenPressed(SetTurret(turret, 270));
//    homeTurret.WhenPressed(HomeTurret(turret));


}