//
// Created by cc on 24/06/22.
//

#ifndef BOTBUSTERS_REBIRTH_MULTIPRESSUREDYNAMICSETSHOOT_H
#define BOTBUSTERS_REBIRTH_MULTIPRESSUREDYNAMICSETSHOOT_H

#include "Math/InterpolatingTable/InterpolatingTable.h"
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Generic/Shooter.h"
#include <units/pressure.h>
#include "Core/VisionManager/VisionManager.h"


struct MultiPressureDynamicSetShootConfig{
    std::vector<std::pair<InterpolatingTable, InterpolatingTable>> tables;//first table is for shooter second table is hood table
    std::vector<double> pressures;
    units::pounds_per_square_inch_t defaultPressure;
};

class MultiPressureDynamicSetShoot : public frc2::CommandHelper<frc2::CommandBase, MultiPressureDynamicSetShoot>{
public:
    MultiPressureDynamicSetShoot(const std::shared_ptr<Shooter> &shooter,
                                 VisionManager * visionManager,
                                 const MultiPressureDynamicSetShootConfig &config);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

    void setPSI(units::pounds_per_square_inch_t psi){
        currentPSI = psi;
    }

    void setPSI(double psi){
        setPSI(units::pounds_per_square_inch_t(psi));
    }

private:
    std::shared_ptr<Shooter> shooter;
    VisionManager * visionManager;
    MultiPressureDynamicSetShootConfig config;

    InterpolatingTable pressureHoodTable;
    InterpolatingTable pressureShooterTable;



    units::pounds_per_square_inch_t currentPSI;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;
    nt::NetworkTableEntry psiTable;
    //testing in simulator
    nt::NetworkTableEntry distanceFromTarget;

    spdlog::logger log{"UseShooter"};
};


#endif //BOTBUSTERS_REBIRTH_MULTIPRESSUREDYNAMICSETSHOOT_H
