//
// Created by alberto on 7/07/19.
//

#include "Core/EctoModule/SystemHandler.h"
#include <sstream>

//TODO Cleanup timing data measurement and implement it for teleop and auto
//Maybe with frc::Tracer(?)
SystemHandler::SystemHandler(const std::string &name) : System(name) {

}

bool SystemHandler::addSubsystem(System &&newSystem) {
    return addSubsystem(std::make_shared<System>(newSystem));
}

bool SystemHandler::addSubsystem(const std::shared_ptr<System> &newSystem) {
	
	if (!newSystem) {
		throw std::runtime_error("Null pointer given");
	}
	
	//Find subsystem in vector
	auto it = find_if(systems.begin(), systems.end(),
	                  [&newSystem](const SystemData &obj) { return obj.subsystem == newSystem; });
	if (it != systems.end()) {
		log->error("Error while adding subcomponent {} to {}, already a subcomponent", newSystem->getName(), getName());
		return false;
	}
	
	SystemData data;
	data.subsystem = newSystem;
	
	systems.emplace_back(data);
	return true;
}

void SystemHandler::robotInit() {
	for (auto &systemData: systems) {
		log->info("Robot Init: {}", systemData.subsystem->getName());
		auto startTime = std::chrono::high_resolution_clock::now();
		systemData.subsystem->robotInit();
		double elapsedTime = std::chrono::duration<double>(
				std::chrono::high_resolution_clock::now() - startTime).count();
		
		systemData.timingData.initRobotTime = elapsedTime;
	}
}

void SystemHandler::robotUpdate() {
	for (auto &systemData: systems) {
		auto startTime = std::chrono::high_resolution_clock::now();
		
		systemData.subsystem->robotUpdate();
		double elapsedTime = std::chrono::duration<double>(
				std::chrono::high_resolution_clock::now() - startTime).count();
		
		systemData.timingData.updateRobotTime += elapsedTime;
		systemData.timingData.updateRobotTime /= 2.0;
		
		systemData.timingData.maxUpdateRobotTime = std::max(systemData.timingData.maxUpdateRobotTime, elapsedTime);
	}
}

void SystemHandler::autoInit() {
    for(auto &systemData : systems){
        systemData.subsystem->autoInit();
    }
}

void SystemHandler::autoUpdate() {
    for(auto &systemData : systems){
        systemData.subsystem->autoUpdate();
    }
}

void SystemHandler::teleopInit() {
    for(auto &systemData : systems){
        systemData.subsystem->teleopInit();
    }
}

void SystemHandler::teleopUpdate() {
    for(auto &systemData : systems){
        systemData.subsystem->teleopUpdate();
    }
}

void SystemHandler::disabledInit() {
	for (auto &systemData: systems) {
		auto startTime = std::chrono::high_resolution_clock::now();
		systemData.subsystem->disabledInit();
		double elapsedTime = std::chrono::duration<double>(
				std::chrono::high_resolution_clock::now() - startTime).count();
		
		systemData.timingData.initDisabledTime = elapsedTime;
	}
}

void SystemHandler::disabledUpdate() {
	for (auto &systemData: systems) {
		auto startTime = std::chrono::high_resolution_clock::now();
		
		systemData.subsystem->disabledUpdate();
		double elapsedTime = std::chrono::duration<double>(
				std::chrono::high_resolution_clock::now() - startTime).count();
		
		systemData.timingData.updateDisabledTime += elapsedTime;
		systemData.timingData.updateDisabledTime /= 2.0;
		
		systemData.timingData.maxUpdateDisabledTime = std::max(systemData.timingData.maxUpdateDisabledTime,
		                                                       elapsedTime);
	}
}

std::vector<std::pair<std::string, TimingData>> SystemHandler::getTimingData() const {
	std::vector<std::pair<std::string, TimingData>> output;
	
	for (auto &systemData: systems) {
		output.emplace_back(std::make_pair(systemData.subsystem->getName(), systemData.timingData));
	}
	
	return output;
}

std::string TimingData::getData() const {
	std::stringstream data;
	data << fmt::format("Robot Init: {}sec Update Update Average: {}sec", initRobotTime, updateRobotTime) << std::endl;
	
	data << fmt::format("Disabled Init: {}sec Update Update Average: {}sec", initDisabledTime, updateDisabledTime)
	     << std::endl;
	
	return data.str();
}
