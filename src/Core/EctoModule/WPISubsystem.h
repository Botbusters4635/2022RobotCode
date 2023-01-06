//
// Created by abiel on 2/27/22.
//

#ifndef BOTBUSTERS_REBIRTH_WPISUBSYSTEM_H
#define BOTBUSTERS_REBIRTH_WPISUBSYSTEM_H

#include "System.h"
#include <frc2/command/SubsystemBase.h>

class WPISubsystem : public System, public frc2::SubsystemBase {
public:
    WPISubsystem(const std::string &name);

    void Init();

    void Periodic() final;
private:
    bool hasInit{false};
};


#endif //BOTBUSTERS_REBIRTH_WPISUBSYSTEM_H
