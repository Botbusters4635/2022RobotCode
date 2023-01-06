//
// Created by cc on 05/02/22.
//

#include "DynamicSetShoot.h"

DynamicSetShoot::DynamicSetShoot(const std::shared_ptr<Shooter> &shooter,
                                 VisionManager *visionManager,
                                 const DynamicSetShootConfig &config) {

    this->shooter = shooter;
    this->visionManager = visionManager;
    this->config = config;
    AddRequirements({shooter.get()});
}

void DynamicSetShoot::Initialize() {
    table = ntInstance.GetTable("DynamicSetShoot");
}

void DynamicSetShoot::Execute() {
    double targetDistance = visionManager->getTargetDistance();
    auto velToTarget = visionManager->getVelocityToTarget();

    double setpointAdj = 00;
    shooter->setSetpoint(config.shooterTable.get(targetDistance) + setpointAdj);
    table->GetEntry("SetpointAdj").SetDouble(setpointAdj);

    shooter->setHood(config.hoodTable.get(targetDistance));
}
void DynamicSetShoot::End(bool interrupted) {
    ;
}
bool DynamicSetShoot::IsFinished() {
    return false;
}

