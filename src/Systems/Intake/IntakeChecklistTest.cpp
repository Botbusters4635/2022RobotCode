//
// Created by abiel on 2/19/22.
//

#include "IntakeChecklistTest.h"

void IntakeChecklistTest::Initialize() {
    ;
}

void IntakeChecklistTest::Execute() {
    if(waitingForInput) return;
    if(state == IntakeChecklistTestState::LowerIntake){
        intake->set(0, true);
        waitForUserInput("Is intake down?");
        state = IntakeChecklistTestState::RaiseIntake;
    } else if(state == IntakeChecklistTestState::RaiseIntake){
        intake->set(0, false);
    }
}

void IntakeChecklistTest::End(bool interrupt) {
    ;
}