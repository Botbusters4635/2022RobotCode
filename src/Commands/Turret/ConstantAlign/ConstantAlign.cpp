//
// Created by cc on 26/07/22.
//

#include "ConstantAlign.h"
#include <frc/smartdashboard/SmartDashboard.h>
ConstantAlign::ConstantAlign(const std::shared_ptr<PIDTurret> &turret,
                             VisionManager *visionManagerIn, const bool useColorSensor) {
    this->turret = turret;
    this->visionManager = visionManagerIn;
    this->useColorSensor = useColorSensor;

    table = ntInstance.GetTable("ColorSensor");
    AddRequirements({turret.get()});

}

void ConstantAlign::Initialize() {
    visionFilter.Reset();
//    movingAvFilter.Reset();


}

void ConstantAlign::Execute() {
    double turretPose = -visionManager->getYawError() - EctoMath::degreesToRadians(45);
    frc::SmartDashboard::PutNumber("turretPoseRaw", turretPose);
    turretPose = visionFilter.Calculate(turretPose);
    frc::SmartDashboard::PutNumber("turretPoseIIRFilter", turretPose);
    turretPose = movingAvFilter.Calculate(turretPose);
    frc::SmartDashboard::PutNumber("moveingFilterOut", turretPose);



    turret->setToRobot(units::radian_t(turretPose));
    lastTurretPose = turretPose;


}
void ConstantAlign::End(bool interrupted) {
    ;
}
bool ConstantAlign::IsFinished() {
    return false;
}