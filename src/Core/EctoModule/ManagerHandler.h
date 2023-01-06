//
// Created by abiel on 2/8/20.
//

#ifndef ECTOMODULE_MANAGERHANDLER_H
#define ECTOMODULE_MANAGERHANDLER_H

#include "Core/EctoModule/Manager.h"
#include <frc/Notifier.h>
#include <list>

class ManagerHandler {
public:
	static ManagerHandler &getInstance() {
		static ManagerHandler instance;
		return instance;
	}
	
	ManagerHandler(ManagerHandler const &) = delete;
	
	ManagerHandler &operator=(ManagerHandler const &) = delete;
	
	void addManager(Manager &updateFunction);

    void init();

    void update();

protected:

	explicit ManagerHandler() {
		;
	};
	
	std::list<Manager*> managerUpdate;
    //frc::Notifier managerUpdateNotifier {[this]{this->update();}};
};


#endif //ECTOMODULE_MANAGERHANDLER_H
