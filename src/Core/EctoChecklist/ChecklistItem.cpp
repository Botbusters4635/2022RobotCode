//
// Created by abiel on 9/29/21.
//

#include "ChecklistItem.h"

ChecklistItem::ChecklistItem(const std::string &name, bool promptUser) : Module(name) {
    this->promptUser = promptUser;
}

void ChecklistItem::assertTest(const std::string &testName, bool result) {
	expectTest(testName, result);
	if (!result) {
		log->error("Halting {} checklist", testName);
		Cancel();
	}
}

void ChecklistItem::expectTest(const std::string &testName, bool result) {
	if (result) {
		//log->info("{}/\"{}\" passed", getName(), testName);
	} else {
		log->error("{}/\"{}\" failed!", getName(), testName);
	}
}

void ChecklistItem::userInputReceived() {
    log->info("Test confirmed!");
    waitingForInput = false;
}

void ChecklistItem::waitForUserInput(const std::string &prompt) {
    if(!waitingForInput) log->info("{}", prompt);
    waitingForInput = true;
}