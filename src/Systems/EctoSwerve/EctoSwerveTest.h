//
// Created by abiel on 2/12/22.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOSWERVETEST_H
#define BOTBUSTERS_REBIRTH_ECTOSWERVETEST_H

#include "Core/EctoChecklist/ChecklistItem.h"

class EctoSwerve;

class EctoSwerveTest : public ChecklistItem {
public:
    EctoSwerveTest(EctoSwerve *swerve);

    void Execute() override;

    bool IsFinished() override;

private:
    EctoSwerve *swerve;
    bool finished{false};
};


#endif //BOTBUSTERS_REBIRTH_ECTOSWERVETEST_H
