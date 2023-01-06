//
// Created by abiel on 4/1/22.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOCHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_ECTOCHARACTERIZATIONROBOT_H

#include "EctoRobot.h"

class EctoCharacterizationRobot : public EctoRobot {
public:
    EctoCharacterizationRobot(const std::string &name) : EctoRobot(name, 5_ms){
        ;
    }
};


#endif //BOTBUSTERS_REBIRTH_ECTOCHARACTERIZATIONROBOT_H
