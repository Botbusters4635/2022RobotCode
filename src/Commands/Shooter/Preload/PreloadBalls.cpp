//
// Created by cc on 07/02/22.
//

#include "PreloadBalls.h"

PreloadBalls::PreloadBalls(const std::shared_ptr<Feeder> &feeder) {
    this->feeder = feeder;
    AddRequirements({feeder.get()});
}

void PreloadBalls::Initialize() {
    hasPressed = false;
}

void PreloadBalls::Execute() {
//    feeder->setFeederVol(12);
    if (!feeder->getFeederLimit()){
        if(!feeder->getMiddleLimit() && !feeder->getIntakeLimit()){
            feeder->setFeederVol(12);
        }
        if(feeder->getMiddleLimit() && !feeder->getIntakeLimit()){
            feeder->setFeederVol(0);
        }
        if(feeder->getIntakeLimit() && feeder->getMiddleLimit()){
            feeder->setFeederVol(6);
        }
        if (!feeder->getMiddleLimit() && feeder->getIntakeLimit()){
            feeder->setFeederVol(12);
        }
    }else{
        feeder->setFeederVol(0);
    }

}

void PreloadBalls::End(bool interrupted){
    feeder->setFeederVol(0);
}

bool PreloadBalls::IsFinished() {
    if (feeder->getFeederLimit() && feeder->getMiddleLimit()){
        return true;
    } else{
        return false;
    }
}