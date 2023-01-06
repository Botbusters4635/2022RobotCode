//
// Created by abiel on 2/27/22.
//

#include "WPISubsystem.h"

WPISubsystem::WPISubsystem(const std::string &name) : System(name) {
    this->SetSubsystem(name);
}

void WPISubsystem::Init() {
    if(!hasInit){
        robotInit();
        hasInit = true;
    }
}

void WPISubsystem::Periodic() {
    Init();
    robotUpdate();
}
