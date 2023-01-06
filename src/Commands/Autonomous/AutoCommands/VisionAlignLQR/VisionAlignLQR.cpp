//
// Created by abiel on 4/12/22.
//

#include "VisionAlignLQR.h"
#include <iostream>

VisionAlignLQR::VisionAlignLQR(const std::shared_ptr<EctoSwerve> &swerve, VisionManager *visionManager, units::second_t waitTime)
         {
    this->visionManager = visionManager;
    this->thetaController = swerve->getThetaController();
    this->waitTime = waitTime;
    frc2::CommandBase::SetName("VisionAlignLQR");

     rotationLqr = std::make_unique<SwerveRotationLQR>(swerve, frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
             0.55757_V / 1_rad_per_s,
             0.039841_V / 1_rad_per_s_sq
     ));
     rotationLqr->reset();

    AddRequirements({thetaController.get()});
}

void VisionAlignLQR::Initialize() {
    thetaController->enable(true);
    atSetpointTimer.Reset();
    atSetpointTimer.Start();
}

void VisionAlignLQR::Execute() {
    auto targetRot = visionManager->getRotationToTarget();
    auto u = rotationLqr->calculate(targetRot.Radians(), 0_rad_per_s);
    thetaController->setTheta(u);
    if(!rotationLqr->atSetpoint()){
        atSetpointTimer.Reset();
    }
}

bool VisionAlignLQR::IsFinished() {
    return rotationLqr->atSetpoint() and atSetpointTimer.Get() > waitTime;
}

void VisionAlignLQR::End(bool interrupted) {
    thetaController->setTheta(0_V);
    thetaController->enable(false);
    atSetpointTimer.Stop();
}