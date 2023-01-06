//
// Created by abiel on 2/4/22.
//

#ifndef BOTBUSTERS_REBIRTH_FORCEATPOSE2D_H
#define BOTBUSTERS_REBIRTH_FORCEATPOSE2D_H

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/drive/Vector2d.h>
#include "Force2d.h"

class ForceAtPose2d {
public:
    Force2d m_force;
    frc::Pose2d m_pos;

    ForceAtPose2d();

    ForceAtPose2d(const Force2d &force, const frc::Pose2d &pos);

    double Torque(const frc::Pose2d &centerOfRotation) const;

    Force2d GetForceInRefFrame(const frc::Pose2d &refFrame) const;

    bool operator==(const ForceAtPose2d &other) const;

    static double VectorCross(const frc::Vector2d &orig, const frc::Vector2d &other){
        return orig.x * other.y - orig.y * other.x;
    }
};

#endif //BOTBUSTERS_REBIRTH_FORCEATPOSE2D_H
