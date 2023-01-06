//
// Created by abiel on 1/9/22.
//

#ifndef BOTBUSTERS_REBIRTH_TRAJECTORYGENERATOR_H
#define BOTBUSTERS_REBIRTH_TRAJECTORYGENERATOR_H

#include <frc/trajectory/TrajectoryGenerator.h>

namespace botbusters {
    class TrajectoryGenerator {
    public:
        static frc::Trajectory GenerateHolonomicTrajectory(
                const frc::Pose2d &start,
                const std::vector<frc::Translation2d> &waypoints,
                const frc::Pose2d &end,
                const frc::TrajectoryConfig &config
                );


        static frc::Trajectory noHeadingTrajectory(const std::vector<frc::Translation2d> &waypoints, const frc::TrajectoryConfig &config);

    };


}

#endif //BOTBUSTERS_REBIRTH_TRAJECTORYGENERATOR_H
