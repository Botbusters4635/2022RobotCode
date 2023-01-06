//
// Created by abiel on 2/4/22.
//

#include "SwerveModuleSim.h"
#include <iostream>
#include <fmt/format.h>

SwerveModuleSim::SwerveModuleSim(const frc::DCMotor &azimuthMotor, const frc::DCMotor &wheelMotor,
                                 units::meter_t wheelRadius, double azimuthGearRatio, double wheelGearRatio,
                                 double azimuthEncGearRatio, double wheelEncGearRatio, double threadStaticCoefFric,
                                 double threadKineticCoefFric, units::newton_t moduleNormalForce,
                                 units::kilogram_square_meter_t azimuthEffectiveMOI) :
        m_azmthMotor(azimuthMotor, azimuthGearRatio, azimuthEffectiveMOI),
        m_wheelMotor(wheelMotor, wheelGearRatio, wheelRadius, m_wheelGearboxLossFactor){

    m_azimuthEncGearRatio = azimuthEncGearRatio;
    m_wheelEncGearRatio = wheelEncGearRatio;
    m_threadStaticFricForce = threadStaticCoefFric * moduleNormalForce;
    m_threadKineticFricForce = threadKineticCoefFric * moduleNormalForce;

    reset(frc::Pose2d());
}

void SwerveModuleSim::setInputVoltages(units::volt_t wheelVoltage, units::volt_t azmthVoltage) {
    m_wheelVoltage = wheelVoltage;
    m_azmthVoltage = azmthVoltage;
}

void SwerveModuleSim::reset(const frc::Pose2d &initModulePose) {
    m_prevModulePose = m_curModulePose = initModulePose;
    m_curLinearSpeed = units::meters_per_second_t(0);
    m_curAzmthAngle = frc::Rotation2d(units::degree_t(0));
}

void SwerveModuleSim::update(units::second_t dt) {
    auto azimuthUnitVec = frc::Vector2d(1,0);
    azimuthUnitVec.Rotate(m_curAzmthAngle.Degrees().value());

    auto velocityAlongAzimuth = units::meters_per_second_t(getModRelTransVel(dt).Dot(azimuthUnitVec));

    m_wheelMotor.update(velocityAlongAzimuth, m_wheelVoltage, dt);
    m_azmthMotor.update(m_azmthVoltage, dt);

    m_curAzmthAngle = frc::Rotation2d(units::degree_t(m_azmthMotor.getMechanisimPositionRev() * 360));
}

frc::Vector2d SwerveModuleSim::getModRelTransVel(units::second_t dt) const {
    units::meters_per_second_t xvel = (m_curModulePose.Translation().X() - m_prevModulePose.Translation().X()) / dt;
    units::meters_per_second_t yvel = (m_curModulePose.Translation().Y() - m_prevModulePose.Translation().Y()) / dt;

    auto moduleTranslationVec = frc::Vector2d(xvel.value(), yvel.value());
    moduleTranslationVec.Rotate(-1.0 * m_curModulePose.Rotation().Degrees().value());
    return moduleTranslationVec;
}

ForceAtPose2d SwerveModuleSim::getCrossThreadFricFoce(const Force2d &netForce, units::second_t dt) {
    auto crossThreadUnitVector = frc::Vector2d(0,1);
    crossThreadUnitVector.Rotate(m_curAzmthAngle.Degrees().value());
    m_crossThreadVelMag = getModRelTransVel(dt).Dot(crossThreadUnitVector);
    m_crossThreadForceMag = netForce.getVector2d().Dot(crossThreadUnitVector);

    Force2d fricForce;

    bool useKinFric = std::abs(m_crossThreadForceMag) > m_threadStaticFricForce.value() ||
            std::abs(m_crossThreadVelMag) > 0.001;

    if(useKinFric){
        m_crossThreadFricForceMag = -1.0 * signum(m_crossThreadVelMag) * m_threadKineticFricForce.value();
    } else {
        m_crossThreadFricForceMag = -1.0 * m_crossThreadForceMag;
    }

    fricForce = Force2d(crossThreadUnitVector);
    fricForce = fricForce * m_crossThreadFricForceMag;

    return {fricForce, m_curModulePose};
}

ForceAtPose2d SwerveModuleSim::getWheelMotiveForce() const {
    return {Force2d(m_wheelMotor.getGroundForce(), m_curAzmthAngle), m_curModulePose};
}

void SwerveModuleSim::setModulePose(const frc::Pose2d &curPos) {
    if(m_prevModulePose == frc::Pose2d()) m_prevModulePose = curPos;
    else m_prevModulePose = m_curModulePose;

    m_curModulePose = curPos;
}

frc::Pose2d SwerveModuleSim::getPose() const {
    return m_curModulePose;
}