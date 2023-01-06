//
// Created by Neil Rodriguez Murillo on 10/8/21.
//

#ifndef BOTBUSTERS_REBIRTH_INTAKE_H
#define BOTBUSTERS_REBIRTH_INTAKE_H

#include <Core/EctoModule/WPISubsystem.h>
#include <Core/PCM/PCMManager.h>
#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
#include <frc/DigitalInput.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <networktables/NetworkTableInstance.h>

struct IntakeConfig {
	std::shared_ptr<frc::DoubleSolenoid> piston;
	std::shared_ptr<EctoMotor> motor;
	
	bool invertMotor = false;
	
	double rampRate = 0.18;
	double currentLimit = 10;

    double beamBreakID;
    bool invertBeamBreak;

};

class Intake : public WPISubsystem {
public:
	explicit Intake(const IntakeConfig &config);

    bool getBeamBreak();

	void set(double pct, bool extended);

    std::vector<std::shared_ptr<ChecklistItem>> createTests() override;

private:
	//MANAGERS
	PCMManager &pcm = PCMManager::getInstance();
	
	//CONFIGS
	IntakeConfig config;
	
	//INSTANCES
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    frc::DigitalInput *beamBreak;

    std::shared_ptr<EctoMotor> motor;

};


#endif //BOTBUSTERS_REBIRTH_INTAKE_H
