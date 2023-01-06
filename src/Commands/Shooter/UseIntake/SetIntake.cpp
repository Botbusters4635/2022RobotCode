//
// Created by Neil Rodriguez Murillo on 10/11/21.
//

#include "SetIntake.h"

SetIntake::SetIntake(const std::shared_ptr<Intake> &intake, double TargetPct, bool state) {
	this->intake = intake;
	this->targetPct = TargetPct;
	this->state = state;
    AddRequirements(intake.get());
}

void SetIntake::Initialize() {
	;
}

void SetIntake::Execute() {
    intake->set(targetPct, state);
}

void SetIntake::End(bool interrupted) {
	;
}

bool SetIntake::IsFinished() {
	return true;
}