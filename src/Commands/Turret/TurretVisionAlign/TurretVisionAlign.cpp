//
// Created by cc on 11/06/22.
//

#include "TurretVisionAlign.h"

TurretVisionAlign::TurretVisionAlign(std::shared_ptr<PIDTurret> & turret) {
    this->turret = turret;
    AddRequirements(turret.get());
    table = ntInstance.GetTable("limelight");
}

void TurretVisionAlign::Initialize() {
    ;
}

void TurretVisionAlign::Execute() {
    bool hasTarget = table->GetNumber("tv", 0) != 0;
    if (hasTarget){
        double setPoint = 0;
        double state = EctoMath::degreesToRadians(table->GetNumber("tx", 0));
        double visionOut = visionPID.Calculate(state, setPoint);
        turret->set(turret->getHeading().value() + visionOut);
    }
}
void TurretVisionAlign::End(bool Interrupted) {
    ;
}
bool TurretVisionAlign::IsFinished() {
    return turret->atGoal();
}