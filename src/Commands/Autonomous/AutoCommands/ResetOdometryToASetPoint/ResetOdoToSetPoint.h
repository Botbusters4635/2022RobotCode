//
// Created by cc on 13/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_RESETODOTOSETPOINT_H
#define BOTBUSTERS_REBIRTH_RESETODOTOSETPOINT_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/EctoSwerve/EctoSwerve.h"

class ResetOdoToSetPoint : public frc2::CommandHelper<frc2::CommandBase, ResetOdoToSetPoint> {
public:
    ResetOdoToSetPoint(const std::shared_ptr<EctoSwerve> &swerve, frc::Pose2d pose2d);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<EctoSwerve> swerve;

    frc::Pose2d pose2d;


};




#endif //BOTBUSTERS_REBIRTH_RESETODOTOSETPOINT_H
