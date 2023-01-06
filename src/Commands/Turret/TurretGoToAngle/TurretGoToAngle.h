//
// Created by cc on 11/06/22.
//

#ifndef BOTBUSTERS_REBIRTH_TURRETGOTOANGLE_H
#define BOTBUSTERS_REBIRTH_TURRETGOTOANGLE_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <Systems/PIDTurret/PIDTurret.h>

class TurretGoToAngle : public frc2::CommandHelper<frc2::CommandBase, TurretGoToAngle>{
public:
    TurretGoToAngle(std::shared_ptr<PIDTurret> &turret, double radians);

    void Initialize() override;
    void Execute() override;
    void End(bool Interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<PIDTurret> turret;
    double radians;


};


#endif //BOTBUSTERS_REBIRTH_TURRETGOTOANGLE_H
