//
// Created by cc on 12/02/22.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOLEDS_H
#define BOTBUSTERS_REBIRTH_ECTOLEDS_H

#include "Core/EctoModule/System.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>


class EctoLeds : public System {
public:
    explicit EctoLeds();

private:

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;


};


#endif //BOTBUSTERS_REBIRTH_ECTOLEDS_H
