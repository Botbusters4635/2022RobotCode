//
// Created by hiram on 28/06/19.
//

#ifndef BOTBUSTERS_REBIRTH_SYSTEM_H
#define BOTBUSTERS_REBIRTH_SYSTEM_H


#include <vector>
#include <memory>
#include "Module.h"
#include "Core/EctoChecklist/ChecklistItem.h"

class System : public Module {
public:
	explicit System(const std::string &name);

    virtual void autoInit(){;};

    virtual void autoUpdate(){;};

    virtual void teleopInit(){;};

    virtual void teleopUpdate(){;};

	virtual void robotInit();

	virtual void robotUpdate();
	
	virtual void disabledInit();
	
	virtual void disabledUpdate();

    ~System();
};


#endif //BOTBUSTERS_REBIRTH_SYSTEM_H
