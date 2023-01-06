//
// Created by cc on 24/11/22.
//

#include "PowerDistributionHub.h"

PowerDistributionHub::PowerDistributionHub(int id, frc::PowerDistribution::ModuleType type) : WPISubsystem("PDH"){
    pd = std::make_unique<frc::PowerDistribution>(id, type);
}

void PowerDistributionHub::setSwitch(bool set) {
    pd->SetSwitchableChannel(set);
}

bool PowerDistributionHub::getSwitch() {
    return pd->GetSwitchableChannel();
}

units::ampere_t PowerDistributionHub::getCurrent(int channel) {
    return units::ampere_t(pd->GetCurrent(channel));
}

void PowerDistributionHub::resetTotals() {
    pd->ResetTotalEnergy();
}

void PowerDistributionHub::robotUpdate() {
    updateTelemetry();
}

void PowerDistributionHub::updateTelemetry() {
    table->GetEntry("Voltage").SetDouble(pd->GetVoltage());
    table->GetEntry("Totals/Power").SetDouble(pd->GetTotalPower());
    table->GetEntry("Totals/Current").SetDouble(pd->GetTotalCurrent());
    table->GetEntry("Totals/Energy").SetDouble(pd->GetTotalEnergy());
    table->GetEntry("temp").SetDouble(pd->GetTemperature());
    table->GetEntry("switchable").SetDouble(pd->GetSwitchableChannel());
}