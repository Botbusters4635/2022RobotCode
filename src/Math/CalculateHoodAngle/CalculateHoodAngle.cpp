//
// Created by cc on 04/02/22.
//

#include "CalculateHoodAngle.h"


CalculateHoodAngle::CalculateHoodAngle(const CalculateHoodAngleConfig &config, const std::shared_ptr<InterpolatingTable> &interpolatingTable){
    this->config = config;
    this->interpolatingTable = interpolatingTable;
    log = std::make_shared<spdlog::logger>("CalculateHoodAngle");

}

double CalculateHoodAngle::calculate(double targetDistance){
    interpolatingTable->addPoint(config.minRange, config.minHoodVal);
    interpolatingTable->addPoint(config.maxRange, config.maxHoodVal);
    if (targetDistance < config.minRange){
        log->error("Distance to target outside of range");
        return config.minHoodVal;

    }else if (targetDistance > config.maxRange){
        log->error("Distance to target outside of range");
        return config.maxHoodVal;
    }else{

        return interpolatingTable->get(targetDistance);
    }


//Do math internally
//    if (targetDistance < config.minRange){
//        log->error("Distance to target outside of range");
//        return config.minHoodVal;
//
//    }else if (targetDistance > config.maxRange){
//        log->error("Distance to target outside of range");
//        return config.maxHoodVal;
//    }else{
//        double angle;
//        angle = config.minHoodVal + ((config.maxHoodVal - config.minHoodVal) / (config.maxRange - config.minRange)) * (targetDistance - config.minRange);
//        return angle;
//    }


}

