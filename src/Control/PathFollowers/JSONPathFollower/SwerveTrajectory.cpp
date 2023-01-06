//
// Created by abiel on 3/29/22.
//

#include "SwerveTrajectory.h"

using State = SwerveTrajectory::State;

State SwerveTrajectory::State::interpolate(const State &end, double x) {
    units::second_t newT = (end.timestamp - timestamp) * x + timestamp;
    const auto startPose = targetPose;
    const auto endPose = end.targetPose;

    frc::Pose2d newPose((endPose.X() - startPose.X()) * x + startPose.X(),
                        (endPose.Y() - startPose.Y()) * x + startPose.Y(),
                        (endPose.Rotation() - startPose.Rotation()) * x + startPose.Rotation());

    const auto startVelocity = targetVelocity;
    const auto endVelocity = end.targetVelocity;

    auto velMult = [](const frc::Velocity2d &vel, double x) {
        return frc::Velocity2d{
                vel.X() * x,
                vel.Y() * x
        };
    };
    frc::Velocity2d newVelocity = velMult((endVelocity - startVelocity), x) + startVelocity;

    units::radians_per_second_t newOmega = ((end.targetOmega - targetOmega) * x) + targetOmega;

    return {
            newT,
            newPose,
            newVelocity,
            newOmega
    };
}

State SwerveTrajectory::getInitialState() const {
    return states.front();
}

units::second_t SwerveTrajectory::getTotalTime() const {
    return states.back().timestamp;
}

State SwerveTrajectory::sample(units::second_t time) {
    if (time < states.front().timestamp) return states.front();
    else if (time > getTotalTime()) return states.back();

    int low = 1;
    int high = states.size() - 1;
    while (low != high) {
        int mid = (low + high) / 2;
        if (states.at(mid).timestamp < time) low = mid + 1;
        else high = mid;
    }

    auto prevState = states.at(low - 1);
    auto currState = states.at(low);

    if (units::math::abs(currState.timestamp - prevState.timestamp).value() < 1E-5) return currState;

    return prevState.interpolate(currState, (time - prevState.timestamp).value() /
                                            (currState.timestamp - prevState.timestamp).value());
}

#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <wpi/json.h>
#include <fstream>
#include <wpi/raw_istream.h>
#include <iostream>

SwerveTrajectory SwerveTrajectory::readJSON(const std::string &pathName) {
    wpi::json root;

    fs::path deployDir = frc::filesystem::GetDeployDirectory();
    deployDir = deployDir / "Paths" / pathName;
    std::error_code ec;
    wpi::raw_fd_istream input{deployDir.string(), ec};
    std::cout << deployDir.string()  << std::endl;
    input >> root;

    auto timestamps = root["timestamp"];
    auto x = root["x"];
    auto y = root["y"];
    auto theta = root["theta"];

    auto vx = root["vx"];
    auto vy = root["vy"];
    auto vtheta = root["omega"];

    std::vector<State> states;
    for (std::size_t i = 0; i < timestamps.size(); i++) {
        states.emplace_back(
                units::second_t(
                        timestamps[i].get<double>()
                        ),
                frc::Pose2d(units::meter_t(x[i].get<double>()),
                            units::meter_t(y[i].get<double>()),
                            {units::radian_t(theta[i].get<double>())}),
                frc::Velocity2d(
                        units::meters_per_second_t(vx[i].get<double>()),
                        units::meters_per_second_t(vy[i].get<double>())
                ),
                 units::radians_per_second_t(
                         vtheta[i].get<double>()
                         )
        );
    }

    return {std::move(states)};
}

frc::Trajectory SwerveTrajectory::getWPITrajectory() const {
    std::vector<frc::Trajectory::State> wpiStates;
    for(const auto &state : states){
        frc::Trajectory::State wState;
        wState.t = state.timestamp;
        wState.velocity = state.targetVelocity.Norm();
        wState.pose = state.targetPose;
        wpiStates.emplace_back(wState);
    }

    return frc::Trajectory(wpiStates);
}