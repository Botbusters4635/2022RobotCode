//
// Created by abiel on 2/10/22.
//

#ifndef BOTBUSTERS_REBIRTH_STOPWATCHCOMMAND_H
#define BOTBUSTERS_REBIRTH_STOPWATCHCOMMAND_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

class StopwatchCommand : public frc2::CommandHelper<frc2::CommandBase, StopwatchCommand> {
public:
    explicit StopwatchCommand(bool startTimer){
        this->startTimer = startTimer;
    }

    void Initialize() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
private:
    bool startTimer = false;
    static units::second_t startTime;
    static bool hasValidStartTime;
};


#endif //BOTBUSTERS_REBIRTH_STOPWATCHCOMMAND_H
