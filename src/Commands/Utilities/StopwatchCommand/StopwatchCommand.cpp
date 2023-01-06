//
// Created by abiel on 2/10/22.
//

#include "StopwatchCommand.h"
#include <frc/Timer.h>
#include <frc2/command/PrintCommand.h>
#include <iostream>

bool StopwatchCommand::hasValidStartTime = false;
units::second_t StopwatchCommand::startTime;

void StopwatchCommand::Initialize() {
    if(startTimer){
        std::cout << "Starting timer" << std::endl;
        StopwatchCommand::startTime = frc::Timer::GetFPGATimestamp();
        StopwatchCommand::hasValidStartTime = true;
    }
}

void StopwatchCommand::End(bool interrupted) {
    std::cout << "Stopwatch ended" << std::endl;
    if(startTimer or !StopwatchCommand::hasValidStartTime) return;
    auto dt = frc::Timer::GetFPGATimestamp() - startTime;
    std::cout << fmt::format("Time elapsed: {}", dt.value()) << std::endl;
    StopwatchCommand::hasValidStartTime = false;
}

bool StopwatchCommand::IsFinished() {
    return true;
}