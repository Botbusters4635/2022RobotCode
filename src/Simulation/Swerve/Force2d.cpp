//
// Created by abiel on 2/4/22.
//

#include "Force2d.h"

Force2d::Force2d() : Force2d(units::newton_t(0), units::newton_t(0)) {
    ;
}

Force2d::Force2d(units::newton_t x, units::newton_t y) {
    m_matrix(0,0) = x.value();
    m_matrix(1,0) = y.value();
}

Force2d::Force2d(units::newton_t mag, const frc::Rotation2d &angle) :
    Force2d(mag * angle.Cos(), mag * angle.Sin()){
    ;
}

Force2d::Force2d(const Eigen::Matrix<double, 2, 1> &m) {
    this->m_matrix = m;
}

Force2d::Force2d(const frc::Vector2d &forceVec) : Force2d(units::newton_t(forceVec.x), units::newton_t(forceVec.y)) {
    ;
}

units::newton_t Force2d::X() const {
    return units::newton_t(m_matrix(0,0));
}

units::newton_t Force2d::Y() const {
    return units::newton_t(m_matrix(1,0));
}

units::newton_t Force2d::Norm() const {
    return units::newton_t(m_matrix.norm());
}

frc::Vector2d Force2d::UnitVector() const {
    return {X() / Norm(), Y() / Norm()};
}

Force2d Force2d::RotateBy(const frc::Rotation2d &angle) const {
    return {
            X() * angle.Cos() - Y() * angle.Sin(),
            X() * angle.Sin() + Y() * angle.Cos()
            };
}

Force2d &Force2d::operator+=(const Force2d &other) {
    m_matrix += other.m_matrix;
    return *this;
}

Force2d Force2d::operator+(const Force2d &other) const {
    return Force2d(*this) += other;
}

Force2d &Force2d::operator-=(const Force2d &other) {
    m_matrix -= other.m_matrix;
    return *this;
}

Force2d Force2d::operator-(const Force2d &other) const {
    return Force2d(*this) -= other;
}

Force2d Force2d::operator-() const {
    return Force2d(-m_matrix);
}

Force2d Force2d::operator*(double scalar) const {
    return Force2d(m_matrix * scalar);
}

Force2d Force2d::operator/(double scalar) const {
    return Force2d(m_matrix / scalar);
}

frc::Vector2d Force2d::getVector2d() const {
    return {X().value(), Y().value()};
}

bool Force2d::operator==(const Force2d &other) const {
    return m_matrix == other.m_matrix;
}