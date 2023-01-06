//
// Created by abiel on 1/26/22.
//

#ifndef BOTBUSTERS_REBIRTH_MOTORROTATIONTEST_H
#define BOTBUSTERS_REBIRTH_MOTORROTATIONTEST_H

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "Core/EctoChecklist/ChecklistItem.h"

enum class MotorControllerSimpleTestState {
    ForwardTest,
    ReverseTest,
    Finished
};

class MotorRotationTest : public ChecklistItem {
public:
    MotorRotationTest(const std::shared_ptr<EctoMotor> &motor);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override {
        return state == MotorControllerSimpleTestState::Finished;
    }
private:
    void runTest(double dt, double setpoint, const std::string &name, MotorControllerSimpleTestState next);

    static constexpr double testDuration = 0.5;
    static constexpr double setpoint = 0.15;
    static constexpr double minimumPositionDelta = 40;
    static constexpr double minimumCurrent = 0.25;

    std::shared_ptr<EctoMotor> motor;

    double startTime{0};
    MotorControllerSimpleTestState state{MotorControllerSimpleTestState::ForwardTest};
    MotorControllerSimpleTestState lastState{MotorControllerSimpleTestState::Finished};
    double initialPosition{0};
};


#endif //BOTBUSTERS_REBIRTH_MOTORROTATIONTEST_H
