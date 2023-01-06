//
// Created by cc on 24/06/22.
//

#include "MultiPressureDynamicSetShoot.h"

MultiPressureDynamicSetShoot::MultiPressureDynamicSetShoot(const std::shared_ptr<Shooter> &shooter,
                                                           VisionManager *visionManager,
                                                           const MultiPressureDynamicSetShootConfig &config) {
    this->shooter = shooter;
    this->visionManager = visionManager;
    this->config = config;
    AddRequirements({shooter.get()});
}

void MultiPressureDynamicSetShoot::Initialize() {
    table = ntInstance.GetTable("MultiPressureDynamicSetShoot");
    psiTable = table->GetEntry("CurrentPSI");
    distanceFromTarget = table->GetEntry("distanceFromTarget");
    distanceFromTarget.SetDouble(0);
    psiTable.SetDouble(config.defaultPressure.value());

    if (config.tables.empty() || config.pressures.empty()) {
        throw std::invalid_argument("not tables or pressures given to multiPressureDynamicSetShoot");
    }

    if (config.tables.size() != config.pressures.size()){
        throw std::invalid_argument("improper pressures");
    }

}

void MultiPressureDynamicSetShoot::Execute() {
    setPSI(units::pounds_per_square_inch_t(psiTable.GetDouble(config.defaultPressure.value())));
    double targetDistance = visionManager->getTargetDistance();
    pressureShooterTable.clearTable();
    pressureHoodTable.clearTable();
//    targetDistance = distanceFromTarget.GetDouble(0);

    int i = 0;
    for (auto & table : config.tables){
        pressureShooterTable.addPoint(config.pressures[i], table.first.get(targetDistance));
        pressureHoodTable.addPoint(config.pressures[i], table.second.get(targetDistance));
        i++;
    }

    shooter->setSetpoint(pressureShooterTable.get(currentPSI.value()));
    shooter->setHood(pressureHoodTable.get(currentPSI.value()));

    table->GetEntry("shooterCalculatedSetPoint").SetDouble(pressureShooterTable.get(currentPSI.value()));
    table->GetEntry("hoodCalculatedSetPoint").SetDouble(pressureHoodTable.get(currentPSI.value()));
    table->GetEntry("targetDistance").SetDouble(targetDistance);

}

void MultiPressureDynamicSetShoot::End(bool interrupted) {
    ;
}

bool MultiPressureDynamicSetShoot::IsFinished() {
    return false;
}