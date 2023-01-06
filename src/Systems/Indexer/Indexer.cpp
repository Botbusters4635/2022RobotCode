//
// Created by cc on 19/01/22.
//

#include "Indexer.h"

Indexer::Indexer(const IndexerConfig &config) : System("Indexer") {
    this->config = config;
    this->motor = config.motor;

    config.motor->deprioritizeUpdateRate();
    config.motor->setControlMode(MotorControlMode::Percent);
    config.motor->setMotorCurrentLimit(config.currentLimit);
    config.motor->enableCurrentLimit(true);
    config.motor->setClosedLoopRampRate(config.rampRate);
    config.motor->enableBrakingOnIdle(config.enableBreakingOnIdle);
    config.motor->invertMotor(config.isInverted);

}

void Indexer::robotInit() {
    ;
}

void Indexer::robotUpdate() {
    ;
}

void Indexer::set(double setSpeed) {
    motor->set(setSpeed);
}