//
// Created by cc on 25/11/22.
//

#include "GearBox.h"

GearBox::GearBox(const GearBoxConfig &config) : WPISubsystem("GearBox") {
    this->config = config;
    this->shifters = config.shifters;

    isEngaged = false;
}


void GearBox::engage(bool set) {
    for (const auto &shifter: shifters){
        PCMManager::set(shifter, set);
    }
    isEngaged = true;
}

bool GearBox::engaged() const{
    return isEngaged;
}

void GearBox::robotUpdate() {
    table->GetEntry("engaged").SetBoolean(isEngaged);
}

