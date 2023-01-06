//
// Created by abiel on 2/20/22.
//

#ifndef BOTBUSTERS_REBIRTH_PIDSHOOTERLEDS_H
#define BOTBUSTERS_REBIRTH_PIDSHOOTERLEDS_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/PIDShooter/PIDShooter.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class PIDShooterLEDs : public frc2::CommandHelper<frc2::CommandBase, PIDShooterLEDs>{
public:
    PIDShooterLEDs(const std::shared_ptr<PIDShooter> &shooter);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override {
        return false;
    }
private:
    std::shared_ptr<PIDShooter> shooter;
    std::shared_ptr<nt::NetworkTable> table;
    const std::string ledTableName = "LEDS";
    const std::string ledEntryName = "LEDEntry";
};


#endif //BOTBUSTERS_REBIRTH_PIDSHOOTERLEDS_H
