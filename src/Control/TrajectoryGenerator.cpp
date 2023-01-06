//
// Created by abiel on 1/9/22.
//

#include "TrajectoryGenerator.h"
#include <cmath>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace botbusters;

frc::Trajectory TrajectoryGenerator::GenerateHolonomicTrajectory(const frc::Pose2d &start,
                                                                 const std::vector<frc::Translation2d> &waypoints,
                                                                 const frc::Pose2d &end,
                                                                 const frc::TrajectoryConfig &config) {
    //Handles special case when there are no interior waypoints, start->end
    const auto poseAfterStart = waypoints.empty() ? end.Translation() : waypoints.front();
    const auto poseBeforeEnd = waypoints.empty() ? start.Translation() : waypoints.back();

    const auto calculate_angles = [](const frc::Translation2d &first, const frc::Translation2d &second){
        const double newAngle = std::atan2(second.Y().value() - first.Y().value(), second.X().value() - first.X().value());
        return frc::Rotation2d(units::radian_t(newAngle));
    };

    std::vector<frc::Pose2d> poses;
    //Add first pose
    poses.emplace_back(start.Translation(), calculate_angles(start.Translation(), poseAfterStart));

    for(auto it = waypoints.begin(); it != waypoints.end(); it++){
        const auto currentWaypoint = *it;
        //Handle case when current waypoint is the last one
        const frc::Translation2d nextWaypoint = it+1 == waypoints.end() ? end.Translation() : *(it+1);

        //Calculate new rotation and create frc::Pose2D
        poses.emplace_back(currentWaypoint, calculate_angles(currentWaypoint, nextWaypoint));
    }

    //Add last pose
    poses.emplace_back(end.Translation(), calculate_angles(poseBeforeEnd, end.Translation()));

    return frc::TrajectoryGenerator::GenerateTrajectory(poses, config);
}


//TODO @gus
frc::Trajectory TrajectoryGenerator::noHeadingTrajectory(const std::vector<frc::Translation2d> &waypoints,
                                                         const frc::TrajectoryConfig &config) {
    if(waypoints.empty()){
        throw std::invalid_argument("No waypoint given to TrajGenerator");
    }
    std::vector<frc::Pose2d> poses;
    for (auto it = waypoints.begin(); it != waypoints.end(); it+1){
        const auto waypoint = *it;

        frc::Translation2d nextWaypoint = it+1 == waypoints.end() ? *waypoints.end() : *(it+1);
//        frc::Translation2d nextWaypoint = waypoint;
        frc::Rotation2d heading = units::radian_t(
                std::atan2((nextWaypoint.Y().value() - waypoint.Y().value()),
                           (nextWaypoint.X().value() - waypoint.X().value())));

        std::cout << heading.Radians().value() << std::endl;
        frc::Pose2d pose{waypoint, heading};
        poses.emplace_back(pose);
    }

    return frc::TrajectoryGenerator::GenerateTrajectory(poses, config);
}