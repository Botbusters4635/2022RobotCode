//
// Created by abiel on 2/26/22.
//

#include "SwerveDistanceAlign.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DigitalInput.h>

SwerveDistanceAlign::SwerveDistanceAlign(const std::shared_ptr<EctoSwerve> &swerve, units::meter_t distance, units::radian_t targetHeading){
    this->swerve = swerve;
    this->targetDistance = distance;
    this->targetRot = frc::Rotation2d(targetHeading);
    this->doHeadingAlign = true;

    AddRequirements(swerve.get());
}

SwerveDistanceAlign::SwerveDistanceAlign(const std::shared_ptr<EctoSwerve> &swerve, units::meter_t distance) :
        SwerveDistanceAlign(swerve, distance, 0_rad){
    this->doHeadingAlign = false;
}

void SwerveDistanceAlign::Initialize(){
    distanceController.Reset(getDistances());
    rotationController.Reset(swerve->getRotation().Radians());
    distanceSlewLimiter.Reset(getDistances());
    distanceSensor.SetAverageBits(16);
}

void SwerveDistanceAlign::Execute() {
    auto rot = swerve->getRotation();
    auto distance = getDistances();
    auto distCalc = distance * units::math::cos(rot.Radians());
    distance = distanceSlewLimiter.Calculate(distCalc);

    nt::NetworkTableInstance::GetDefault().GetEntry("SwerveDistanceAlign/Distance").SetDouble(distance.value());
    nt::NetworkTableInstance::GetDefault().GetEntry("SwerveDistanceAlign/CalcDistance").SetDouble(distCalc.value());

    distanceController.SetGoal(targetDistance);
    rotationController.SetGoal(0_rad);

    auto distOut = distanceController.Calculate(distance);
    auto rotCalc = rotationController.Calculate(rot.Radians());

    nt::NetworkTableInstance::GetDefault().GetEntry("SwerveDistanceAlign/OutputVel").SetDouble(distCalc.value());

    frc::ChassisSpeeds vels;
    vels.vy = 0_mps; vels.vx = units::meters_per_second_t (distOut);
    vels.omega = units::radians_per_second_t (doHeadingAlign ? rotCalc : 0);
    swerve->setVoltage(vels);
}

units::meter_t SwerveDistanceAlign::getDistances() {
    double sensorRaw = distanceSensor.GetAverageVoltage();
    nt::NetworkTableInstance::GetDefault().GetEntry("SwerveDistanceAlign/SensorRaw").SetDouble(sensorRaw);

//    units::second_t pulseLen = sensorRaw * (1_s/distanceSensor->());
//    double pulse = units::convert<units::second, units::microsecond>(pulseLen).value();
//    validPulse = pulse >= 300.0 or pulse <= 5000.0;
//    pulse = std::clamp(pulse, 300.0, 5000.0);

    return units::millimeter_t((sensorRaw / ((4.88/5.0)/1000.0))); //1 us -> 1 mm
}

bool SwerveDistanceAlign::IsFinished() {
    auto distance = getDistances();
    return units::math::abs(distance - targetDistance) < 0.1_m && (frc::Rotation2d(swerve->getRotation()) - targetRot).Radians() < 0.1_rad;
}

void SwerveDistanceAlign::End(bool interrupted) {
    swerve->setVoltage({});
}