//
// Created by abiel on 1/9/22.
//

#include "ChecklistRunner.h"
#include <stdexcept>
#include "Core/EctoInput/InputManager.h"

void ChecklistRunner::robotInit() {
    //Select the first system to run
    nextItem();
    InputManager::getInstance().registerButton(nextButton, "A");
}

void ChecklistRunner::robotUpdate() {
    if(hasFinished) {
        if(doFinishedOutput){
            log->info("Finished running all tests!");
            doFinishedOutput = false;
        }
        return;
    }
    if(systemItems.empty()){
        return;
    }
    auto checklistItem = systemItems.at(currentSystem)[currentChecklistItem];

    if(checklistItem->IsFinished()){
        log->info("Finished with: {}", checklistItem->getName());
        checklistItem->End(false);
        nextItem();
        hasUserAccepted = false;
        didUserButtonPrompt = false;
    } else {
        if(!checklistItem->userPromptNeeded() or hasUserAccepted){
            if(!hasInitialized){
                checklistItem->Initialize();
                hasInitialized = true;
            }
            checklistItem->Execute();

            if(checklistItem->waitingForUserInput() && !lastNextButton && nextButton.get()){
                checklistItem->userInputReceived();
            }

        } else {
            if(!didUserButtonPrompt){
                log->info("Waiting for confirm for: {}", checklistItem->getName());
                didUserButtonPrompt = true;
            }

            if(!lastNextButton && nextButton.get()){
                hasUserAccepted = true;
            }
        }
    }

    lastNextButton = nextButton.get();
}

void ChecklistRunner::addItems(const std::string &system, const std::vector<std::shared_ptr<ChecklistItem>> &items, int priority) {
    if(systemItems.find(system) != systemItems.end()) throw std::runtime_error("Duplicate systems added to checklist");
    systemItems[system] = items;
    systemOrder.emplace(priority, system);
}

void ChecklistRunner::nextItem() {
    currentChecklistItem++;

    if(currentSystem == "NULL" || currentChecklistItem == systemItems.at(currentSystem).size()){
        //All checklist items have been executed, switch to testing another system
        if(systemOrder.empty()){
            hasFinished = true;
            return;
        }

        currentSystem = systemOrder.top().second;
        systemOrder.pop();

        currentChecklistItem = 0;
    }
}