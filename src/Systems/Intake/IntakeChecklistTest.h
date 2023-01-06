//
// Created by abiel on 2/19/22.
//

#ifndef BOTBUSTERS_REBIRTH_INTAKECHECKLISTTEST_H
#define BOTBUSTERS_REBIRTH_INTAKECHECKLISTTEST_H

#include "Intake.h"
#include "Core/EctoChecklist/ChecklistItem.h"

enum class IntakeChecklistTestState {
    LowerIntake,
    RaiseIntake,
    Finished
};

class IntakeChecklistTest : public ChecklistItem {
public:
    IntakeChecklistTest(Intake *intake) : ChecklistItem("IntakeTest"){
        this->intake = intake;
    }

    void Initialize() override;

    void Execute() override;

    void End(bool interrupt) override;

    bool IsFinished() override {
        return state == IntakeChecklistTestState::Finished;
    }

private:
    Intake *intake;
    IntakeChecklistTestState state = IntakeChecklistTestState::LowerIntake;
};


#endif //BOTBUSTERS_REBIRTH_INTAKECHECKLISTTEST_H
