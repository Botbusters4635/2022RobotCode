//
// Created by cc on 06/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_GOTOANGLE_H
#define BOTBUSTERS_REBIRTH_GOTOANGLE_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <frc/controller/ProfiledPIDController.h>

class GoToAngle : public frc2::CommandHelper<frc2::CommandBase, GoToAngle> {
public:
	GoToAngle(const std::shared_ptr<EctoSwerve> &swerve, double angle, double tol = 0.1);

	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	frc::ChassisSpeeds chassisSpeeds;
	frc::ProfiledPIDController<units::radians> anglePID{0.9, 0.08, 0.1, {units::radians_per_second_t((2 * M_PI) * 1.5), units::radians_per_second_squared_t((2 * M_PI) * 6)}};
	double state;
	double angle;
	double pidOut;
	double tol;
	
	
};

#endif //BOTBUSTERS_REBIRTH_GOTOANGLE_H
