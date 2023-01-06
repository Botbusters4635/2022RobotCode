//
// Created by abiel on 2/4/22.
//

#include "ForceAtPose2d.h"

ForceAtPose2d::ForceAtPose2d() : ForceAtPose2d(Force2d(), frc::Pose2d()) {
    ;
}

ForceAtPose2d::ForceAtPose2d(const Force2d &force, const frc::Pose2d &pos) {
    m_force = force;
    m_pos = pos;
}

double ForceAtPose2d::Torque(const frc::Pose2d &centerOfRotation) const {
    auto transCORtoF = frc::Transform2d(centerOfRotation, m_pos);

    Force2d alignedForce = GetForceInRefFrame(centerOfRotation);

    auto leverArm = frc::Vector2d(transCORtoF.X().value(), transCORtoF.Y().value());
    return VectorCross(leverArm, alignedForce.getVector2d());
}

Force2d ForceAtPose2d::GetForceInRefFrame(const frc::Pose2d &refFrame) const {
    auto trans = frc::Transform2d(refFrame, m_pos);
    return m_force.RotateBy(trans.Rotation());
}

bool ForceAtPose2d::operator==(const ForceAtPose2d &other) const {
    return std::abs((m_force.X() - other.m_force.X()).value()) < 1E-9
        && std::abs((m_force.Y() - other.m_force.Y()).value()) < 1E-9
        && std::abs((m_pos.X() - other.m_pos.X()).value()) < 1E-9
        && std::abs((m_pos.Y() - other.m_pos.Y()).value()) < 1E-9;
}