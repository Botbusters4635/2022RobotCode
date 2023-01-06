//
// Created by cc on 05/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_SPINUPSHOOTER_H
#define BOTBUSTERS_REBIRTH_SPINUPSHOOTER_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Systems/Generic/Shooter.h"
#include "Core/VisionManager/VisionManager.h"

#include "Math/InterpolatingTable/InterpolatingTable.h"

#include "spdlog/spdlog.h"

class SpinupShooter : public frc2::CommandHelper<frc2::CommandBase, SpinupShooter>{
public:
    SpinupShooter(const std::shared_ptr<Shooter> &shooter, double defautlVel);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<Shooter> shooter;

    double defaultValue;

};


#endif //BOTBUSTERS_REBIRTH_SPINUPSHOOTER_H
