//
// Created by cc on 19/01/22.
//

#include "Feeder.h"

Feeder::Feeder(const FeederConfig &config) : WPISubsystem("Feeder"){
    this->config = config;
    this->feederMotor = config.feederMotor;

    intakeLimit = new frc::DigitalInput(config.limitSwitchIds[0]);
    middleLimit = new frc::DigitalInput(config.limitSwitchIds[1]);
    feederLimit = new frc::DigitalInput(config.limitSwitchIds[2]);

#ifndef SIMULATION
    counter = std::make_unique<frc::Counter>(feederLimit);
    counter->SetUpSourceEdge(true, false);
    digitalGlitchFilter.SetPeriodNanoSeconds(1850000);
    digitalGlitchFilter.Add(counter.get());
#endif

    table = ntInstance.GetTable("Feeder");
    feederVel = table->GetEntry("Velocity");
    limitTopState = table->GetEntry("intakeLimitState");
    limitBottomState = table->GetEntry("middleLimitState");
    limitMiddleState = table->GetEntry("feederLimit");
    feederAppliedVoltage = table->GetEntry("appliedVoltageFeeder");

    ballsShot = table->GetEntry("Balls Shot");

    current = table->GetEntry("Motor Current");

    config.feederMotor->deprioritizeUpdateRate();
    config.feederMotor->setControlMode(MotorControlMode::Voltage);
    config.feederMotor->setMotorCurrentLimit(config.currentLimit);
    config.feederMotor->enableCurrentLimit(true);
    config.feederMotor->setClosedLoopRampRate(config.rampRate);
    config.feederMotor->enableBrakingOnIdle(config.enableBreakingOnIdle);
    config.feederMotor->invertMotor(config.isInverted);
    config.feederMotor->setFeedbackMode(MotorFeedbackMode::QuadEncoder);



}

void Feeder::robotInit() {
    ;
}

void Feeder::robotUpdate() {
    limitTopState.SetBoolean(getIntakeLimit());
    limitBottomState.SetBoolean(getMiddleLimit());
    limitMiddleState.SetBoolean(getFeederLimit());
    ballsShot.SetDouble(getBallsShot());
    table->GetEntry("FeederCurrent").SetDouble(feederMotor->getCurrent());




}

void Feeder::setFeederVol(double setVoltage) {
    feederAppliedVoltage.SetDouble(setVoltage);
    feederMotor->set(setVoltage, MotorControlMode::Voltage);
}

bool Feeder::getIntakeLimit() {
    bool state = intakeLimit->Get();
    if (config.invertedSwitches[0]){
        state = !state;
    }

    return state;
}

bool Feeder::getMiddleLimit() {
    bool state = middleLimit->Get();
    if(config.invertedSwitches[1]){
        state = !state;
    }return state;
}

bool Feeder::getFeederLimit() {
    bool state = feederLimit->Get();
    if (config.invertedSwitches[2]){
       state = !state;
    }return state;

}

void Feeder::resetBallsShot() {
#ifndef SIMULATION
    counter->Reset();
#endif
}

int Feeder::getBallsShot() {
#ifndef SIMULATION
    return counter->Get();
#else
    return 0;
#endif
}