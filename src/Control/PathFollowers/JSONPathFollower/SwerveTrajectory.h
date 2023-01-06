//
// Created by abiel on 3/29/22.
//

#ifndef BOTBUSTERS_REBIRTH_SWERVETRAJECTORY_H
#define BOTBUSTERS_REBIRTH_SWERVETRAJECTORY_H

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Velocity2d.h>
#include <frc/trajectory/Trajectory.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <vector>

class SwerveTrajectory {
public:
    struct State {
        State(units::second_t timestamp,
              const frc::Pose2d &pose,
              const frc::Velocity2d &vel,
              units::radians_per_second_t omega){
            this->timestamp = timestamp;
            this->targetPose = pose;
            this->targetVelocity = vel;
            this->targetOmega = omega;
        }

        units::second_t timestamp;
        frc::Pose2d targetPose;
        frc::Velocity2d targetVelocity;
        units::radians_per_second_t targetOmega;

        State interpolate(const State &end, double x);
    };

    SwerveTrajectory(const std::vector<State> &&states){
        this->states = std::move(states);
    };

    State getInitialState() const;

    units::second_t getTotalTime() const;

    State sample(units::second_t time);

    static SwerveTrajectory readJSON(const std::string &pathName);

    frc::Trajectory getWPITrajectory() const;

private:
    std::vector<State> states;
};


#endif //BOTBUSTERS_REBIRTH_SWERVETRAJECTORY_H
