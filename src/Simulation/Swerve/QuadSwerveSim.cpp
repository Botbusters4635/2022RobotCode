//
// Created by abiel on 2/4/22.
//

#include "QuadSwerveSim.h"

#include <utility>

QuadSwerveSim::QuadSwerveSim(units::meter_t wheelBaseWidth, units::meter_t wheelBaseLength, units::kilogram_t robotMass,
                             units::kilogram_square_meter_t robotMOI, std::array<SwerveModuleSim, 4> &&modules) : m_modules(std::move(modules)) {
    m_robotToModuleTL = {
            frc::Translation2d(wheelBaseWidth / 2, wheelBaseLength / 2),
            frc::Translation2d(wheelBaseWidth / 2, -wheelBaseLength / 2),
            frc::Translation2d(-wheelBaseWidth / 2, wheelBaseLength / 2),
            frc::Translation2d(-wheelBaseWidth / 2, -wheelBaseLength / 2)
    };

    m_robotToModule = {
            frc::Transform2d(m_robotToModuleTL.at(FL), frc::Rotation2d()),
            frc::Transform2d(m_robotToModuleTL.at(FR), frc::Rotation2d()),
            frc::Transform2d(m_robotToModuleTL.at(BL), frc::Rotation2d()),
            frc::Transform2d(m_robotToModuleTL.at(BR), frc::Rotation2d())
    };

    m_robotMass = robotMass;
    m_robotMOI = robotMOI;
}

void QuadSwerveSim::modelReset(const frc::Pose2d &pose) {
    m_accelPrev = frc::Vector2d();
    m_velPrev = frc::Vector2d();
    m_rotAccelPrev = units::radians_per_second_squared_t (0);
    m_rotVelPrev = units::radians_per_second_t (0);
    int i = 0;
    for(auto &module : m_modules){
        module.reset(pose.TransformBy(m_robotToModule.at(i)));
        i++;
    }

    m_curPose = pose;
}

void QuadSwerveSim::update(units::second_t dt) {
    frc::Pose2d fieldRF;
    frc::Transform2d fieldToRobot(fieldRF, m_curPose);

    for(int i = 0; i < 4; i++){
        auto tmp = fieldRF.TransformBy(fieldToRobot);
        auto modPose = tmp.TransformBy(m_robotToModule.at(i));
        m_modules.at(i).setModulePose(modPose);
        m_modules.at(i).update(dt);
    }

    std::array<ForceAtPose2d, 4> wheelMotiveForces;
    for(int i = 0; i < 4; i++){
        wheelMotiveForces[i] = m_modules.at(i).getWheelMotiveForce();
    }

    Force2d preFricNetForce(units::newton_t(0), units::newton_t(0));
    for(auto &mf : wheelMotiveForces){
        preFricNetForce += mf.GetForceInRefFrame(m_curPose);
    }

    Force2d sidekickForce(units::newton_t(0), units::newton_t(0));
    preFricNetForce += sidekickForce;

    ForceAtPose2d pfRobotForce(preFricNetForce, m_curPose);

    std::array<ForceAtPose2d, 4> xtreadFrifFrc;
    for(int i = 0; i < 4; i++){
        SwerveModuleSim &mod = m_modules.at(i);
        double ffrac = 1.0 / 4.0;
        auto pfModForce = pfRobotForce.GetForceInRefFrame(mod.getPose()) * ffrac;
        xtreadFrifFrc[i] = mod.getCrossThreadFricFoce(pfModForce, dt);
    }

    Force2d forceOnRobotCenter = preFricNetForce;

    for(const auto &force : xtreadFrifFrc){
        forceOnRobotCenter += force.GetForceInRefFrame(m_curPose);
    }

    ForceAtPose2d netForce(forceOnRobotCenter, m_curPose);
    auto robotForceInFieldRefFrame = netForce.GetForceInRefFrame(fieldRF);

    double netTorque{0};
    for(int i = 0; i < 4; i++){
        netTorque += wheelMotiveForces.at(i).Torque(m_curPose);
        netTorque += xtreadFrifFrc.at(i).Torque(m_curPose);
    }

    auto accel = (robotForceInFieldRefFrame * (1.0/m_robotMass.value())).getVector2d();

    frc::Vector2d velocity(m_velPrev.x + (accel.x + m_accelPrev.x) / 2.0 * dt.value(),
                           m_velPrev.y + (accel.y + m_accelPrev.y) / 2.0 * dt.value());

    frc::Translation2d posChange(units::meter_t(velocity.x + m_velPrev.x) / 2.0 * dt.value(),
                                 units::meter_t((velocity.y + m_velPrev.y) / 2.0 * dt.value()));

    auto rotAccel = units::radians_per_second_squared_t(netTorque / m_robotMOI.value());
    units::radians_per_second_t rotVel = m_rotVelPrev + (rotAccel + m_rotAccelPrev) / 2.0 * dt;
    units::radian_t rotPosChange = (rotVel + m_rotVelPrev) / 2.0 * dt;

    m_velPrev = velocity;
    m_accelPrev = accel;
    m_rotVelPrev = rotVel;
    m_rotAccelPrev = rotAccel;

    posChange = posChange.RotateBy(-m_curPose.Rotation());

    frc::Twist2d motionThisLoop;
    motionThisLoop.dx = posChange.X();
    motionThisLoop.dy = posChange.Y();
    motionThisLoop.dtheta = rotPosChange;

    m_curPose = m_curPose.Exp(motionThisLoop);
}