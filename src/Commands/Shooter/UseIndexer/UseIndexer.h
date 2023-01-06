//
// Created by cc on 25/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_USEINDEXER_H
#define BOTBUSTERS_REBIRTH_USEINDEXER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Indexer/Indexer.h"

class UseIndexer : public frc2::CommandHelper<frc2::CommandBase, UseIndexer>{
public:
    explicit UseIndexer(const std::shared_ptr<Indexer> &indexer, double targetVel, bool isInverted);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<Indexer> indexer;
    double targetVel;
    bool isInverted;

};


#endif //BOTBUSTERS_REBIRTH_USEINDEXER_H
