//
// Created by tener on 10/12/21.
//

#ifndef BOTBUSTERS_REBIRTH_SETINTAKE_H
#define BOTBUSTERS_REBIRTH_SETINTAKE_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Intake/Intake.h"

class SetIntake : public frc2::CommandHelper<frc2::CommandBase, SetIntake> {
public:
	
	explicit SetIntake(const std::shared_ptr<Intake> &intake, double TargetPct = 1.00, bool state = false);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<Intake> intake;
	double targetPct;
	bool state;
};

#endif //BOTBUSTERS_REBIRTH_SETINTAKE_H
