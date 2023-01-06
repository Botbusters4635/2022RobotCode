//
// Created by cc on 05/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_DYNAMICSETSHOOT_H
#define BOTBUSTERS_REBIRTH_DYNAMICSETSHOOT_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Systems/Generic/Shooter.h"
#include "Core/VisionManager/VisionManager.h"

#include "Math/InterpolatingTable/InterpolatingTable.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <spdlog/spdlog.h>

struct DynamicSetShootConfig{
    InterpolatingTable hoodTable, shooterTable;
};

class DynamicSetShoot : public frc2::CommandHelper<frc2::CommandBase, DynamicSetShoot>{
public:
    DynamicSetShoot(const std::shared_ptr<Shooter> &shooter,
                    VisionManager * visionManager,
                    const DynamicSetShootConfig &config);


    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<Shooter> shooter;
    VisionManager * visionManager;
    DynamicSetShootConfig config;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    spdlog::logger log{"UseShooter"};
};


#endif //BOTBUSTERS_REBIRTH_DYNAMICSETSHOOT_H
