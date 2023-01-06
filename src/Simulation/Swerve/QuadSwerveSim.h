//
// Created by abiel on 2/4/22.
//

#ifndef BOTBUSTERS_REBIRTH_QUADSWERVESIM_H
#define BOTBUSTERS_REBIRTH_QUADSWERVESIM_H

#include "SwerveModuleSim.h"
#include <array>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/torque.h>

class QuadSwerveSim {
public:
    QuadSwerveSim(units::meter_t wheelBaseWidth, units::meter_t wheelBaseLength, units::kilogram_t robotMass,
                  units::kilogram_square_meter_t robotMOI, std::array<SwerveModuleSim, 4> &&modules);

    void modelReset(const frc::Pose2d &pose);

    void update(units::second_t dt);

    frc::Pose2d getCurrentPose() const{
        return m_curPose;
    }

    SwerveModuleSim &getModule(size_t i){
        return m_modules.at(i);
    }

private:
    const int FL = 0;
    const int FR = 1;
    const int BL = 2;
    const int BR = 3;

    std::array<SwerveModuleSim, 4> m_modules;

    frc::Vector2d m_accelPrev{};
    frc::Vector2d m_velPrev{};

    units::radians_per_second_squared_t m_rotAccelPrev{0};
    units::radians_per_second_t m_rotVelPrev{0};

    std::array<frc::Translation2d, 4> m_robotToModuleTL;
    std::array<frc::Transform2d, 4> m_robotToModule;

    frc::Pose2d m_curPose{};

    units::kilogram_t m_robotMass;
    units::kilogram_square_meter_t m_robotMOI;
};


#endif //BOTBUSTERS_REBIRTH_QUADSWERVESIM_H
