//
// Created by cc on 24/11/22.
//

#ifndef BOTBUSTERS_REBIRTH_POWERDISTRIBUTIONHUB_H
#define BOTBUSTERS_REBIRTH_POWERDISTRIBUTIONHUB_H

#include <frc/PowerDistribution.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <Core/EctoModule/WPISubsystem.h>
#include <units/current.h>

class PowerDistributionHub : public WPISubsystem {
public:
    PowerDistributionHub(int id, frc::PowerDistribution::ModuleType type);

    void robotUpdate() override;

//    void autoInit() override;
//
//    void autoUpdate() override;
//
//    void teleopInit() override;
//
//    void teleopUpdate() override;

    void updateTelemetry();

    units::ampere_t getCurrent(int channel);

    void setSwitch(bool set);

    bool getSwitch();

    void resetTotals();

private:
    std::unique_ptr<frc::PowerDistribution> pd;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("PowerDistribution");

};


#endif //BOTBUSTERS_REBIRTH_POWERDISTRIBUTIONHUB_H
