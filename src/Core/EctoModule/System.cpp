//
// Created by hiram on 28/06/19.
//

#include "Core/EctoModule/System.h"

System::System(const std::string &name) : Module(name) {
    ;
}

void System::robotInit() {
	;
}

void System::robotUpdate() {
	;
}

void System::disabledInit() {
	;
}

void System::disabledUpdate() {
	;
}

#include <iostream>
System::~System(){
    std::cout << fmt::format("Destructor: {} called, ohgodno", getName()) << std::endl;
}
