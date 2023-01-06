//
// Created by abiel on 2/12/22.
//

#include "EctoSwerveTest.h"
#include "Systems/EctoSwerve/EctoSwerve.h"

EctoSwerveTest::EctoSwerveTest(EctoSwerve *swerve) : ChecklistItem("EctoSwerveTest", false) {
    this->swerve = swerve;
}

void EctoSwerveTest::Execute() {
    std::array<frc::SwerveModuleState, 4> states;
    for(auto &state : states){
        state.speed = 0.1_mps;
        state.angle = 0_rad;
    }

    swerve->setModules(states, false);
    waitForUserInput("Confirm all swerve modules are working!");
    finished = true;
}

bool EctoSwerveTest::IsFinished() {
    return finished;
}