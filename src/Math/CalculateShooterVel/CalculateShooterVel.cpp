//
// Created by cc on 05/02/22.
//

#include "CalculateShooterVel.h"

CalculateShooterVel::CalculateShooterVel(const CalculateShooterVelConfig &config, const std::shared_ptr<InterpolatingTable> &interpolatingTable) {
    this->config = config;
    this->interpolatingTable = interpolatingTable;
}

double CalculateShooterVel::calculate(double targetDistance) {
    interpolatingTable->addPoint(config.minRange, config.minVel);
    interpolatingTable->addPoint(config.maxRange, config.maxVel);

    if(targetDistance < config.minRange){
        return config.minVel;

    }else if (targetDistance >  config.maxRange){
        return config.maxVel;
    }else{
        return 0;
    }
}


