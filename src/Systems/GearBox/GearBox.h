//
// Created by cc on 25/11/22.
//

#ifndef BOTBUSTERS_REBIRTH_GEARBOX_H
#define BOTBUSTERS_REBIRTH_GEARBOX_H


#include "Core/EctoModule/WPISubsystem.h"
#include <frc/DoubleSolenoid.h>
#include "Core/PCM/PCMManager.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

struct GearBoxConfig{
    std::vector<std::shared_ptr<frc::DoubleSolenoid>> shifters;

    double engagedReduction = 0;
    double disengagedReduction = 0;

};

class GearBox : public WPISubsystem{
public:
    GearBox(const GearBoxConfig &config);

    void robotUpdate() override;

    void engage(bool set);

    [[nodiscard]] bool engaged() const;

private:
    PCMManager &pcm = PCMManager::getInstance();

    GearBoxConfig config;

    std::vector<std::shared_ptr<frc::DoubleSolenoid>> shifters;

    bool isEngaged{false};

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("GearBox");

};


#endif //BOTBUSTERS_REBIRTH_GEARBOX_H
