//
// Created by abiel on 1/26/22.
//

#include "MotorRotationTest.h"
#include <frc/Timer.h>

MotorRotationTest::MotorRotationTest(const std::shared_ptr<EctoMotor> &motor) :
        ChecklistItem(fmt::format("MotorControllerSimpleTest_{}", motor->getName()), false){
    this->motor = motor;
    //Just in case
    if(!motor->doBasicTests())
        throw std::runtime_error("Motor cannot run basic tests safely!");
}

void MotorRotationTest::Initialize() {
    startTime = frc::Timer::GetFPGATimestamp().value();
    state = MotorControllerSimpleTestState::ForwardTest;
    initialPosition = motor->getPosition();
}

void MotorRotationTest::runTest(double dt, double setpoint, const std::string &name, MotorControllerSimpleTestState next) {
    motor->set(setpoint, MotorControlMode::Percent);
    if(dt >= testDuration){
        //Check motor and move on to next phase
        //log->info("position: {}, current: {}", motor->getPosition() - initialPosition, motor->getCurrent());
        bool testResult = std::fabs(motor->getPosition() - initialPosition) > minimumPositionDelta;
        if(!testResult){
            log->info("position: {}, velocity: {}, current: {}", motor->getPosition() - initialPosition, motor->getVelocity(), motor->getCurrent());
        }
        expectTest(fmt::format("{} encoder movement", name), testResult);

        //expectTest(fmt::format("{} current", name), motor->getCurrent() > minimumCurrent);

        startTime = frc::Timer::GetFPGATimestamp().value();
        state = next;
        initialPosition = motor->getPosition();
    }
}

void MotorRotationTest::Execute() {
    if(lastState != state) startTime = frc::Timer::GetFPGATimestamp().value();
    auto dt = frc::Timer::GetFPGATimestamp().value() - startTime;

    if(state == MotorControllerSimpleTestState::ForwardTest){
        runTest(dt, setpoint, "Forwards", MotorControllerSimpleTestState::ReverseTest);
    } else if(state == MotorControllerSimpleTestState::ReverseTest){
        runTest(dt, -setpoint, "Reverse", MotorControllerSimpleTestState::Finished);
    }

    lastState = state;
}

void MotorRotationTest::End(bool interrupted) {
    motor->set(0);
}