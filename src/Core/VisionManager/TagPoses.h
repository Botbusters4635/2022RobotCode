//
// Created by cc on 14/11/22.
//

#ifndef BOTBUSTERS_REBIRTH_TAGPOSES_H
#define BOTBUSTERS_REBIRTH_TAGPOSES_H

#include <frc/geometry/Pose2d.h>
#include <bits/stdc++.h>

namespace aprilTags{
    struct TagPoses{
        std::map<int, frc::Pose2d> tag36h11;
        std::map<int, frc::Pose2d> tag25h9;
        std::map<int, frc::Pose2d> tag16h5;
    };
}



#endif //BOTBUSTERS_REBIRTH_TAGPOSES_H
