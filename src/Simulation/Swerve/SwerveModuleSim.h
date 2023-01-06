//
// Created by abiel on 2/4/22.
//

#ifndef BOTBUSTERS_REBIRTH_SWERVEMODULESIM_H
#define BOTBUSTERS_REBIRTH_SWERVEMODULESIM_H

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/drive/Vector2d.h>
#include "SimpleMotorWithMassModel.h"
#include "MotorGearboxWheelSim.h"
#include "ForceAtPose2d.h"

//https://github.com/wpilibsuite/allwpilib/pull/3374/files
class SwerveModuleSim {
public:
    SwerveModuleSim(){;}

    SwerveModuleSim(
            const frc::DCMotor &azimuthMotor,
            const frc::DCMotor &wheelMotor,
            units::meter_t wheelRadius,
            double azimuthGearRatio,
            double wheelGearRatio,
            double azimuthEncGearRatio,
            double wheelEncGearRatio,
            double threadStaticCoefFric,
            double threadKineticCoefFric,
            units::newton_t moduleNormalForce,
            units::kilogram_square_meter_t azimuthEffectiveMOI
            );

    void setInputVoltages(units::volt_t wheelVoltage, units::volt_t azmthVoltage);

    units::volt_t getWheelVoltage() const {
        return m_wheelVoltage;
    }

    units::volt_t getAzmthVoltage() const{
        return m_azmthVoltage;
    }

    double getAzimuthEncoderPositionRev() const {
        return m_azmthMotor.getMechanisimPositionRev() * m_azimuthEncGearRatio;
    }

    double getWheelEncoderPositionRev() const {
        return m_wheelMotor.getPositionRev() * m_wheelEncGearRatio;
    }

    units::radians_per_second_t getWheelMotorVelocity() const {
        return m_wheelMotor.getMotorSpeed();
    }

    void reset(const frc::Pose2d &initModulePose);

    void update(units::second_t dt);

    frc::Vector2d getModRelTransVel(units::second_t dt) const;

    ForceAtPose2d getCrossThreadFricFoce(const Force2d &netForce, units::second_t dt);

    ForceAtPose2d getWheelMotiveForce() const;

    void setModulePose(const frc::Pose2d &curPos);

    frc::Pose2d getPose() const;

private:
    SimpleMotorWithMassModel m_azmthMotor;
    MotorGearboxWheelSim m_wheelMotor;

    double m_azimuthEncGearRatio;
    double m_wheelEncGearRatio;
    units::newton_t m_threadStaticFricForce;
    units::newton_t m_threadKineticFricForce;

    static constexpr double m_wheelGearboxLossFactor = 0.01;

    frc::Pose2d m_prevModulePose{}, m_curModulePose{};

    units::meters_per_second_t m_curLinearSpeed;

    frc::Rotation2d m_curAzmthAngle = frc::Rotation2d();

    double m_crossThreadFricForceMag = 0;
    double m_crossThreadVelMag = 0;
    double m_crossThreadForceMag = 0;

    units::volt_t m_wheelVoltage;
    units::volt_t m_azmthVoltage;

    static double signum(double n){
        if(n > 0) return 1.0;
        if(n < 0) return -1.0;
        return 0;
    }
};


#endif //BOTBUSTERS_REBIRTH_SWERVEMODULESIM_H
