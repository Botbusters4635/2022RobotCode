//
// Created by abiel on 2/4/22.
//

#ifndef BOTBUSTERS_REBIRTH_FORCE2D_H
#define BOTBUSTERS_REBIRTH_FORCE2D_H

#include <Eigen/Core>
#include <units/force.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/drive/Vector2d.h>

class Force2d {
public:
    Eigen::Matrix<double, 2, 1> m_matrix;

    Force2d();

    Force2d(units::newton_t x, units::newton_t y);

    Force2d(units::newton_t mag, const frc::Rotation2d &angle);

    explicit Force2d(const Eigen::Matrix<double, 2, 1> &m);

    Force2d(const frc::Vector2d &forceVec);

    units::newton_t X() const;

    units::newton_t Y() const;

    units::newton_t Norm() const;

    frc::Vector2d UnitVector() const;

    Force2d RotateBy(const frc::Rotation2d &angle) const;

    Force2d &operator+=(const Force2d &other);

    Force2d operator+(const Force2d &other) const;

    Force2d &operator-=(const Force2d &other);

    Force2d operator-(const Force2d &other) const;

    Force2d operator-() const;

    Force2d operator*(double scalar) const;

    Force2d operator/(double scalar) const;

    bool operator==(const Force2d &other) const;

    frc::Vector2d getVector2d() const;
};


#endif //BOTBUSTERS_REBIRTH_FORCE2D_H
