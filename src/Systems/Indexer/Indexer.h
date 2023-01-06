//
// Created by cc on 19/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_INDEXER_H
#define BOTBUSTERS_REBIRTH_INDEXER_H

#include "Core/EctoModule/System.h"
#include "Core/MotorHandler/EctoMotor/EctoMotor.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

struct IndexerConfig{

    std::shared_ptr<EctoMotor> motor;

    bool isInverted = false;

    double rampRate = 0.18;

    double currentLimit = 15;

    MotorControlMode motorControlMode;

    bool enableBreakingOnIdle;

    double gearReduction;



};

class Indexer : public System{
public:
    explicit Indexer(const IndexerConfig &config);

    void robotInit() override;
    void robotUpdate() override;

    void set(double setSpeed);

private:
    IndexerConfig config;

    std::shared_ptr<EctoMotor> motor;
};


#endif //BOTBUSTERS_REBIRTH_INDEXER_H
