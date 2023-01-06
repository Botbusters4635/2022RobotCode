//
// Created by abiel on 2/1/22.
//

#ifndef BOTBUSTERS_REBIRTH_WAITFORBUTTON_H
#define BOTBUSTERS_REBIRTH_WAITFORBUTTON_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Core/EctoInput/Buttons/EctoButton.h"
#include "Core/EctoInput/InputManager.h"
#include <networktables/NetworkTableInstance.h>

class WaitForButton : public frc2::CommandHelper<frc2::CommandBase, WaitForButton> {
public:
    WaitForButton(const std::string &buttonName, int joystickId = 0);

    void Initialize() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:

    nt::NetworkTableEntry waitingEntry = nt::NetworkTableInstance::GetDefault().GetTable("waitforButton")->GetEntry("waiting");
    std::string buttonName;
    int joystickId;
    EctoButton button;
};


#endif //BOTBUSTERS_REBIRTH_WAITFORBUTTON_H
