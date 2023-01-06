//
// Created by cc on 29/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_RELEASECLIMBER_H
#define BOTBUSTERS_REBIRTH_RELEASECLIMBER_H


#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/PIDClimber/PIDClimber.h"
#include <frc/Timer.h>
#include <optional>
class ReleaseClimber : public frc2::CommandHelper<frc2::CommandBase, ReleaseClimber>{
public:
    explicit ReleaseClimber(const std::shared_ptr<PIDClimber> &climber, bool state);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<PIDClimber> climber;
    bool state;
    double startTime{};


};


#endif //BOTBUSTERS_REBIRTH_RELEASECLIMBER_H
