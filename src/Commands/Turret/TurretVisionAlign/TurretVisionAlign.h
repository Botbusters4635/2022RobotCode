//
// Created by cc on 11/06/22.
//

#ifndef BOTBUSTERS_REBIRTH_TURRETVISIONALIGN_H
#define BOTBUSTERS_REBIRTH_TURRETVISIONALIGN_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <Systems/PIDTurret/PIDTurret.h>
#include <Core/VisionManager/VisionManager.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

class TurretVisionAlign : public frc2::CommandHelper<frc2::CommandBase, TurretVisionAlign>{
public:
    TurretVisionAlign(std::shared_ptr<PIDTurret> &turret);

    void Initialize() override;
    void Execute() override;
    void End(bool Interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<PIDTurret> turret;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;
    nt::NetworkTableEntry visionError, hasTarget;

    frc::PIDController visionPID{1.2, 0, 0.000045};
};


#endif //BOTBUSTERS_REBIRTH_TURRETVISIONALIGN_H
