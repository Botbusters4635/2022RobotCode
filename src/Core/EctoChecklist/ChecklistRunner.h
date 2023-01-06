//
// Created by abiel on 1/9/22.
//

#ifndef BOTBUSTERS_REBIRTH_CHECKLISTRUNNER_H
#define BOTBUSTERS_REBIRTH_CHECKLISTRUNNER_H

#include "Core/EctoModule/System.h"
#include "Core/EctoInput/Buttons/EctoButton.h"
#include "ChecklistItem.h"
#include <queue>
#include <map>

class ChecklistRunner : public System {
public:
    ChecklistRunner() : System("ChecklistRunner"){;}

    void robotInit() override;

    void robotUpdate() override;

    void addItems(const std::string &system, const std::vector<std::shared_ptr<ChecklistItem>> &items, int priority = 0);

private:
    std::string currentSystem{"NULL"};
    size_t currentChecklistItem;

    EctoButton nextButton;

    //Groups checklist items by systems
    std::map<std::string, std::vector<std::shared_ptr<ChecklistItem>>> systemItems;
    std::priority_queue<std::pair<int, std::string>> systemOrder; //Greater number -> executed first

    void nextItem(); //Switches over to a new item

    bool hasFinished = false;
    bool lastNextButton = false;
    bool hasUserAccepted = false;
    bool doFinishedOutput = false;
    bool didUserButtonPrompt = false;
    bool hasInitialized = false;
};


#endif //BOTBUSTERS_REBIRTH_CHECKLISTRUNNER_H
