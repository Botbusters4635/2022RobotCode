//
// Created by cc on 18/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_LOADINTAKE_H
#define BOTBUSTERS_REBIRTH_LOADINTAKE_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <Systems/Intake/Intake.h>
#include <frc/Timer.h>

#include <Commands/Shooter/UseIntake/SetIntake.h>

class LoadIntake : public frc2::CommandHelper<frc2::CommandBase, LoadIntake>{
public:
    explicit LoadIntake(const std::shared_ptr<Intake> &intake);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;


private:

    std::shared_ptr<Intake> intake;
    bool initialState{};

};


#endif //BOTBUSTERS_REBIRTH_LOADINTAKE_H
