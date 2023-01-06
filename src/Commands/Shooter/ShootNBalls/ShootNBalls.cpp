//
// Created by abiel on 2/7/22.
//

#include "ShootNBalls.h"
#include <frc/Timer.h>

ShootNBalls::ShootNBalls(const std::shared_ptr<Feeder> &feeder, int balls){
    this->feeder = feeder;
    nBalls = balls;
    AddRequirements(feeder.get());
}

void ShootNBalls::Initialize(){
    initialCount = feeder->getBallsShot();
    lastDelayCount = initialCount;
}

void ShootNBalls::Execute() {


    if(feeder->getBallsShot() != lastDelayCount){
        //Do delay
        feeder->setFeederVol(0);
        if (frc::Timer::GetFPGATimestamp() - delayStartTime >= 100_ms){
            //End delay
            lastDelayCount = feeder->getBallsShot();
        }
    } else {
        feeder->setFeederVol(12);
        delayStartTime = frc::Timer::GetFPGATimestamp();
    }

    if ((feeder->getBallsShot() - initialCount) >= nBalls && !hasRun){
        startTime = frc::Timer::GetFPGATimestamp();
        hasRun = true;
    }

}

void ShootNBalls::End(bool interrupted){
    feeder->setFeederVol(0);
}

bool ShootNBalls::IsFinished(){
#ifdef SIMULATION
    return frc::Timer::GetFPGATimestamp() - startTime >= units::second_t(1) * nBalls;
#endif
    return feeder->getBallsShot() - initialCount >= nBalls && ((frc::Timer::GetFPGATimestamp().value() - startTime.value()) > 0.0);
}