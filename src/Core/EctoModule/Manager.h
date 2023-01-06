//
// Created by hiram on 28/06/19.
//

#ifndef BOTBUSTERS_REBIRTH_MANAGER_H
#define BOTBUSTERS_REBIRTH_MANAGER_H


#include "Module.h"

class Manager : public Module {
public:
	Manager(Manager const &) = delete;
	
	Manager &operator=(Manager const &) = delete;
    virtual void init() = 0;
    virtual void update() = 0;

protected:
	explicit Manager(const std::string &name) : Module(name) {
	};
};


#endif //BOTBUSTERS_REBIRTH_MANAGER_H
