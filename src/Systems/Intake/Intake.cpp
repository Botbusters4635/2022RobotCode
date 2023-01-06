//
// Created by Neil Rodriguez Murillo on 10/8/21.
//

#include "Intake.h"
#include "IntakeChecklistTest.h"

Intake::Intake(const IntakeConfig &config) : WPISubsystem("Intake") {
	this->config = config;
    this->motor = config.motor;

    beamBreak = new frc::DigitalInput(config.beamBreakID);

	motor->deprioritizeUpdateRate();
	motor->setOpenLoopRampRate(config.rampRate);
	motor->setMotorCurrentLimit(config.currentLimit);
	motor->enableCurrentLimit(true);
    motor->enableBrakingOnIdle(false);
	motor->invertMotor(config.invertMotor);

    table = ntInstance.GetTable("Intake");
}

bool Intake::getBeamBreak(){
    bool state = beamBreak->Get();
    if (config.invertBeamBreak){
        return !state;
    }else{return state;}
}

void Intake::set(double pct, bool extended) {
//    auto vol = xRightJoy.get();
//    auto state = leftShoulder.get();
	motor->set(pct, MotorControlMode::Percent);
	PCMManager::set(config.piston, extended);
    table->GetEntry("PistonState").SetBoolean(extended);
    table->GetEntry("MotorState").SetDouble(pct);
    table->GetEntry("Current").SetDouble(config.motor->getCurrent());
}

std::vector<std::shared_ptr<ChecklistItem>> Intake::createTests() {
    return {std::make_shared<IntakeChecklistTest>(this)};
}

