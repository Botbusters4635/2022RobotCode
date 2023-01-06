//
// Created by abiel on 2/20/22.
//

#include "PIDShooterLEDs.h"

PIDShooterLEDs::PIDShooterLEDs(const std::shared_ptr<PIDShooter> &shooter) {
    this->shooter = shooter;
}

void PIDShooterLEDs::Initialize() {
    table = nt::NetworkTableInstance::GetDefault().GetTable(ledTableName);
}

void PIDShooterLEDs::Execute() {
    bool flywheelReady = shooter->flywheelAtSetpoint();
    bool hoodReady = shooter->hoodAtSetpoint();

    //table->GetEntry(ledEntryName).SetBoolean(flywheelReady && hoodReady);
    table->GetEntry(ledEntryName).SetBoolean(false);
}

void PIDShooterLEDs::End(bool interrupted) {
    table->GetEntry(ledEntryName).SetBoolean(false);
}