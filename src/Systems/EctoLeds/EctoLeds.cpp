//
// Created by cc on 12/02/22.
//

#include "EctoLeds.h"

EctoLeds::EctoLeds() : System("EctoLeds") {
    table = ntInstance.GetTable("EctoLeds");

}