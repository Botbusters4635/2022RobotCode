//
// Created by abiel on 8/22/19.
//

#ifndef BOTBUSTERSREBIRTH_ECTOPCM_H
#define BOTBUSTERSREBIRTH_ECTOPCM_H

#include <frc/DoubleSolenoid.h>
#include <Core/EctoModule/Manager.h>
#include <string>
#include <map>
#include <list>
#include <frc/Compressor.h>
#include <units/pressure.h>

struct PistonInfo {
	PistonInfo() {
		;
	}
	
	PistonInfo(const std::string &name, const int CANid, const frc::PneumaticsModuleType type,  int id1, int id2) {
		this->name = name;
        this->CANid = CANid;
        this->type = type;
		this->id1 = id1;
		this->id2 = id2;
	}
	
	std::string name{"NA"};
	int id1{-1}, id2{-1};
    int CANid{-1};
    frc::PneumaticsModuleType type{frc::PneumaticsModuleType::REVPH};
};

class PCMManager : public Manager {
public:
    static PCMManager &getInstance() {
        static PCMManager instance;
        return instance;
    }

    std::shared_ptr<frc::DoubleSolenoid> getPiston(const std::string &name);
	
	void init() override;
	
	void setPCMConfig(const std::list<PistonInfo> &pistonInfo);
	
	//Wrapper for legacy code
	static void set(const std::shared_ptr<frc::DoubleSolenoid> &solenoid, bool state) {
		auto val = state ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse;
		solenoid->Set(val);
	}

	void update() override;

    units::pounds_per_square_inch_t getAnalogPressure();

private:
	PCMManager();
	
	bool hasInit = false;
	
	PCMManager &operator=(const PCMManager &);
	
	void initializePistons();

    frc::Compressor compressor{20, frc::PneumaticsModuleType::REVPH};
	std::list<PistonInfo> pistonInfo;
	std::map<std::string, std::shared_ptr<frc::DoubleSolenoid>> pistons;
};

#endif //BOTBUSTERSREBIRTH_ECTOPCM_H
