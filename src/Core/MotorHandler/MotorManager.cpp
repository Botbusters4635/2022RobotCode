//
// Created by alberto on 31/07/19.
//

#include "MotorManager.h"
#include "EctoMotor/EctoSpark.h"
#include "EctoMotor/EctoPWM.h"
#include "EctoMotor/EctoSimulatedMotor.h"
#include "ChecklistTests/MotorRotationTest.h"
#include <iostream>


MotorManager::MotorManager() : Manager("MotorHandler") {
	log->info("Initializing EctoMotors...");
}

void MotorManager::init() {
	initializeMotors();
	hasInit = true;
}

void MotorManager::setMotorInfo(const std::list<MotorInfo> &motorInfo) {
	this->motorInfo = motorInfo;
}

void MotorManager::initializeMotors() {
	for (const auto &motorData: motorInfo) {
		std::shared_ptr<EctoMotor> motor;
		std::string fwVer;
#ifndef SIMULATION
		switch (motorData.type) {
			case EctoMotorType::SparkMax:
				motor = std::make_shared<EctoSpark>(motorData.id, motorData.name, false, motorData.runTests);
				fwVer = motor->getFirmwareVersion();
				if (fwVer != sparkMaxVersion) {
					log->error("SparkMax: {} with name: {} has different FW version: {}", motorData.id, motorData.name,
					           fwVer);
				}
				break;
				//TODO define other motors
			default:
				throw std::runtime_error("Invalid motor type defined!");
		}
#else
		motor = std::make_shared<EctoSimulatedMotor>(motorData.id, motorData.name);
#endif
		motorControllers.emplace_back(motor);
	}
}

void MotorManager::update() {
	;
}

std::shared_ptr<EctoMotor> MotorManager::getMotor(const std::string &name) {
	if (!hasInit) throw std::runtime_error("MotorManager has not been initialized!");
	for (auto &motor: motorControllers) {
		//log->info("Motor Name: {}", motor->getName());
		if (motor->getName() == name) {
			return motor;
		}
	}
	throw std::invalid_argument("Could not get motor with the name '" + name + "', it does not exist");
}

std::shared_ptr<EctoSimulatedMotor> MotorManager::getSimulatedMotor(const std::string &name) {
    if (!hasInit) throw std::runtime_error("MotorManager has not been initialized!");

    for (auto &motor: motorControllers) {
        //log->info("Motor Name: {}", motor->getName());
        if (motor->getName() == name && motor->getMotorType() == EctoMotorType::Simulated) {
            return std::dynamic_pointer_cast<EctoSimulatedMotor>(motor);
        }
    }
    throw std::invalid_argument("Could not get simulated motor with the name '" + name + "', it does not exist");
}

std::vector<std::shared_ptr<ChecklistItem>> MotorManager::createTests() {
    std::vector<std::shared_ptr<ChecklistItem>> out;
    for(auto motor : motorControllers){
        out.emplace_back(std::make_shared<MotorControllerFaultTest>(
                motor
                ));

        if(motor->doBasicTests())
            out.emplace_back(std::make_shared<MotorRotationTest>(
                    motor
                    ));
    }

    return out;
}
