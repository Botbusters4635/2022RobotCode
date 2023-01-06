//
// Created by abiel on 2/6/20.
//

#include "LedManager.h"

LEDManager::LEDManager() : Manager("LEDManager") {
}

void LEDManager::init() {
    table = ntInstance.GetTable(baseTableKey);

}

void LEDManager::update() {
	if (commandQueue.count(PatternPriority::HighPriority) != 0) {
		//High priority
		sendCommand(commandQueue.at(PatternPriority::HighPriority));
		//log->info("High");
	} else if (commandQueue.count(PatternPriority::MedPriority) != 0) {
		//Med priority
		sendCommand(commandQueue.at(PatternPriority::MedPriority));
		//log->info("Med");
	} else if (commandQueue.count(PatternPriority::LowPriority) != 0) {
		//Low priority
		sendCommand(commandQueue.at(PatternPriority::LowPriority));
		//log->info("Low");
	}
}

void LEDManager::queueCommand(const PatternCommand &command, PatternPriority priority) {
	commandQueue[priority] = command;
}

void LEDManager::stopCommandsByPriority(PatternPriority priority) {
	auto iter = commandQueue.find(priority);
	if (iter != commandQueue.end()) {
		commandQueue.erase(iter);
	}
}

void LEDManager::sendCommand(const PatternCommand &command) {

	if (command.primaryPattern == LEDPattern::GreenFire or command.primaryPattern == LEDPattern::BlueFire or
	    command.primaryPattern == LEDPattern::RedFire) {

		int firePaletteNumber = 0;
		switch (command.primaryPattern) {
			case LEDPattern::GreenFire:
				firePaletteNumber = 0;
				break;
			
			case LEDPattern::BlueFire:
				firePaletteNumber = 2;
				break;
			
			case LEDPattern::RedFire:
				firePaletteNumber = 1;
				break;
			default:
				break;
		}
		
		table->GetEntry(fmt::format("{}", firePalette)).SetDouble(firePaletteNumber);
	} else {
		table->GetEntry(fmt::format("{}", isFireEffectKey)).SetDouble(0);
		
		table->GetEntry(fmt::format("{}/{}", firstKey, effectKey)).SetDouble(
				static_cast<double>(command.primaryPattern));
		table->GetEntry(fmt::format("{}/{}", secondKey, effectKey)).SetDouble(
				static_cast<double>(command.secondaryPattern));
		
		table->GetEntry(fmt::format("{}", breatheRateKey)).SetDouble(command.patternRate);
	}
}
