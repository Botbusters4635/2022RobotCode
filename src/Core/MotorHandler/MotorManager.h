//
// Created by alberto on 31/07/19.
//
#ifndef ECTOCONTROL_MOTORHANDLER_H
#define ECTOCONTROL_MOTORHANDLER_H

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "EctoMotor/DataTypes/EctoMotorType.h"
#include <Core/EctoModule/Manager.h>
#include <list>
#include "Core/EctoChecklist/ChecklistItem.h"
#include "EctoMotor/EctoSimulatedMotor.h"

struct MotorInfo {
	MotorInfo(EctoMotorType type, const std::string &name, int id, bool runTests = false) {
		this->type = type;
		this->name = name;
		this->id = id;
        this->runTests = runTests;
	}
	
	EctoMotorType type;
	std::string name;
	int id;
    bool runTests;
};

class MotorControllerFaultTest : public ChecklistItem{
public:
    MotorControllerFaultTest(std::shared_ptr<EctoMotor> motor)
        : ChecklistItem(fmt::format("MotorControllerTest_{}_{}", motor->getName(), motor->getId()), false){
        this->motor = motor;
    }

    void Execute() override {
        assertTest("Motor Controller has no faults", motor->runFaultTest());
        hasFinished = true;
    }

    bool IsFinished() override {
        return hasFinished;
    }
private:
    std::shared_ptr<EctoMotor> motor;
    bool hasFinished = false;
};

class MotorManager : public Manager {

public:
    static MotorManager &getInstance() {
        static MotorManager instance;
        return instance;
    }

	std::shared_ptr<EctoMotor> getMotor(const std::string &name);

    std::shared_ptr<EctoSimulatedMotor> getSimulatedMotor(const std::string &name);
	
	void init() override;
	
	void update() override;
	
	void setMotorInfo(const std::list<MotorInfo> &motorInfo);

    std::vector<std::shared_ptr<ChecklistItem>> createTests() override;
private:
	bool hasInit = false;
	
	const std::string talonVersion = "20.1";
	const std::string sparkMaxVersion = "v1.5.2";
	
	std::list<MotorInfo> motorInfo;
	
	void initializeMotors();
	
	std::vector<std::shared_ptr<EctoMotor>> motorControllers;
	
	MotorManager();
    MotorManager &operator=(const MotorManager &);

};

#endif //ECTOCONTROL_MOTORHANDLER_H
