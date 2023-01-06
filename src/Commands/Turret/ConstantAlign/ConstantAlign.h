//
// Created by cc on 26/07/22.
//

#ifndef BOTBUSTERS_REBIRTH_CONSTANTALIGN_H
#define BOTBUSTERS_REBIRTH_CONSTANTALIGN_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Utilities/WPI/SlewRateLimiter/RateLimiter.h"

#include <networktables/NetworkTable.h>

#include "Systems/PIDTurret/PIDTurret.h"
#include "Core/VisionManager/VisionManager.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/MedianFilter.h>

class ConstantAlign : public frc2::CommandHelper<frc2::CommandBase, ConstantAlign>{
public:
    ConstantAlign(const std::shared_ptr<PIDTurret> &turret,  VisionManager *visionManagerIn, const bool useColorSensor);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<PIDTurret> turret;
    VisionManager * visionManager;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    frc::LinearFilter<double> visionFilter = frc::LinearFilter<double>::SinglePoleIIR(0.2, 20_ms);

    frc::LinearFilter<double> movingAvFilter = frc::LinearFilter<double>::MovingAverage(5);
    double lastTurretPose{};
    bool useColorSensor;

};


#endif //BOTBUSTERS_REBIRTH_CONSTANTALIGN_H
