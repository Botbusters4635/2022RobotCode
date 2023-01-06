//
// Created by cc on 07/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_PRELOADBALLS_H
#define BOTBUSTERS_REBIRTH_PRELOADBALLS_H


#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Systems/Indexer/Indexer.h"
#include "Systems/Feeder/Feeder.h"

class PreloadBalls : public frc2::CommandHelper<frc2::CommandBase, PreloadBalls>{
public:
    explicit PreloadBalls(const std::shared_ptr<Feeder> &feeder);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    std::shared_ptr<Feeder> feeder;

    bool hasPressed = false;
};



#endif //BOTBUSTERS_REBIRTH_PRELOADBALLS_H
