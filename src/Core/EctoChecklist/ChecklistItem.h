//
// Created by abiel on 9/29/21.
//

#ifndef BOTBUSTERS_REBIRTH_CHECKLISTITEM_H
#define BOTBUSTERS_REBIRTH_CHECKLISTITEM_H

#include "Core/EctoModule/Module.h"
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

class ChecklistItem : public frc2::CommandHelper<frc2::CommandBase, ChecklistItem>, public Module {
public:
    //TODO make it so that the user is prompted before test runs, instead of after
    /*
     * If prompt user == true, button input will be required for the next test to be ran
     */
	ChecklistItem(const std::string &name, bool promptUser = true);

protected:
	/**
	 * Stops running test if test did not pass
	 */
	void assertTest(const std::string &testName, bool result);
	
	/**
	 * Keeps running test, even if it did not pass
	 */
	void expectTest(const std::string &testName, bool result);

    void waitForUserInput(const std::string &prompt = "Press button to confirm test!");

    bool waitingForInput{false};

private:
    friend class ChecklistRunner;

    void userInputReceived();

    bool userPromptNeeded() const {return promptUser; };

    bool waitingForUserInput() const { return waitingForInput; };

    bool promptUser;
};

class ChecklistTest : public ChecklistItem {
public:
    ChecklistTest() : ChecklistItem("ChecklistTest") {
        ;
    }

    void Execute() override {
        assertTest("Passes when true", true);
        hasFinished = true;
    }

    bool IsFinished() override { return hasFinished; }

private:
    bool hasFinished = false;
};

#endif //BOTBUSTERS_REBIRTH_CHECKLISTITEM_H
