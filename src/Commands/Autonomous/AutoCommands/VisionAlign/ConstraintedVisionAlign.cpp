////
//// Created by cc on 06/09/22.
////
//
//#include "ConstraintedVisionAlign.h"
//
//#include <iostream>
//#include <frc/smartdashboard/SmartDashboard.h>
//
//ConstraintedVisionAlign::ConstraintedVisionAlign(const std::shared_ptr<EctoSwerve> &swerve,
//                         VisionManager *visionManager, units::second_t waitTime, units::radians_per_second maxVel, units::radians_per_second_squared maxAcc) {
//    this->visionManager = visionManager;
//    this->swerve = swerve;
//    atSetpointWaitTime = waitTime;
//    this->maxVel = maxVel;
//    this->maxAcc = maxAcc;
//
//    AddRequirements(swerve.get());
//}
//
//void ConstraintedVisionAlign::Initialize() {
//    visionAnglePID.EnableContinuousInput(-M_PI, M_PI);
//    visionAnglePID.Reset();
//    lastState = 0;
//    lastTime = frc::Timer::GetFPGATimestamp();
//
//    atSetpointTimer.Reset();
//    atSetpointTimer.Start();
//}
//
////Switch to using pose estimated vision alignment
//void ConstraintedVisionAlign::Execute() {
//    auto dt = frc::Timer::GetFPGATimestamp() - lastTime;
//    auto state = visionManager->getYawError();
//    if(state != lastState)
//        stateVel = units::radians_per_second_t((state - lastState) / dt.value());
//
//    visionOut = visionAnglePID.Calculate(state, 0);
//    chassisSpeeds.omega = units::radians_per_second_t(visionOut);
//    swerve->setPercent(chassisSpeeds);
//
//    if(!isReady()){
//        atSetpointTimer.Reset();
//    }
//
//    lastTime = frc::Timer::GetFPGATimestamp();
//    lastState = state;
//}
//
//void ConstraintedVisionAlign::End(bool interrupted) {
//    chassisSpeeds.omega = units::radians_per_second_t(0);
//    swerve->setPercent(chassisSpeeds);
//    atSetpointTimer.Stop();
//
//}
//
//bool ConstraintedVisionAlign::isReady() const {
//    auto state = visionManager->getYawError();
//    //std::cout << fmt::format("Setpoint: {}, State: {}, Error: {} Tol: {} StateVel: {}", setPoint, state, std::abs(setPoint - state), tol, stateVel.value()) << std::endl;
//
//    auto atSetpoint = std::abs(setPoint - state) < tol;
//    auto stateVelStab = std::abs(stateVel.value()) < 0.05;
//
//    return atSetpoint && stateVelStab;
//}
//
//bool ConstraintedVisionAlign::IsFinished() {
//    return isReady() and atSetpointTimer.Get() > atSetpointWaitTime;
//}