//
// Created by abiel on 2/8/20.
//

#include "ManagerHandler.h"

void ManagerHandler::addManager(Manager &manager) {
    managerUpdate.emplace_back(&manager);

}

void ManagerHandler::init() {
    for (auto& manager: managerUpdate) {
        manager->init();
    }
    //managerUpdateNotifier.StartPeriodic(20_ms);
}

void ManagerHandler::update() {
    for (auto& manager: managerUpdate) {
        manager->update();
    }
}
