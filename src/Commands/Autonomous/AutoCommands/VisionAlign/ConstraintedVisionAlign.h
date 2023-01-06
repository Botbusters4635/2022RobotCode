////
//// Created by cc on 06/09/22.
////
//
//#ifndef BOTBUSTERS_REBIRTH_CONSTRAINTEDVISIONALIGN_H
//#define BOTBUSTERS_REBIRTH_CONSTRAINTEDVISIONALIGN_H
//
//#include <frc2/command/CommandBase.h>
//#include <frc2/command/CommandHelper.h>
//#include <frc/Timer.h>
//#include "Systems/EctoSwerve/EctoSwerve.h"
//#include "Core/VisionManager/VisionManager.h"
//#include <limits>
//
//class ConstraintedVisionAlign : public frc2::CommandHelper<frc2::CommandBase, ConstraintedVisionAlign> {
//public:
//    ConstraintedVisionAlign(const std::shared_ptr<EctoSwerve> &swerve, VisionManager *visionManager, units::second_t waitTime = 100_ms, units::radians_per_second maxVel, units::radians_per_second_squared maxAcc);
//
//    void Initialize() override;
//
//    void Execute() override;
//
//    void End(bool interrupted) override;
//
//    bool IsFinished() override;
//
//
//private:
//    bool isReady() const;
//
//    std::shared_ptr<EctoSwerve> swerve;
//    VisionManager *visionManager;
//    frc::ChassisSpeeds chassisSpeeds;
//
//    double setPoint = 0;
//    double visionOut{};
//    bool isFinished{};
//    double tol = 0.05;
//    double angle, swerveAngle;
//
//    double lastState;
//    units::second_t lastTime;
//    units::radians_per_second_t stateVel;
//
//    units::radians_per_second maxVel;
//    units::radians_per_second_squared maxAcc;
//
//    frc2::PIDController visionAnglePID{1.025, 0.00, 0.066};
//
//    frc::Timer atSetpointTimer;
//    units::second_t atSetpointWaitTime;
//};
//
//#endif //BOTBUSTERS_REBIRTH_CONSTRAINTEDVISIONALIGN_H
