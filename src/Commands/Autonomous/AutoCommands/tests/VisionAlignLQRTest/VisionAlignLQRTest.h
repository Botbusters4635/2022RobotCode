//
// Created by cc on 13/04/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONALIGNLQRTEST_H
#define BOTBUSTERS_REBIRTH_VISIONALIGNLQRTEST_H


#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>


#include <frc/trajectory/Trajectory.h>
#include "Control/TrajectoryGenerator.h"
#include  <frc/trajectory/TrajectoryConfig.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/Feeder/Feeder.h"
#include "Systems/Intake/Intake.h"
#include "Systems/Generic/Shooter.h"


#include "Control/PathFollowers/HolonomicPathFollower.h"

#include "Control/PathFollowers/PathFollowerRotationProfile.h"

#include "Commands/Autonomous/AutoCommands/ResetWheelStatesToZero/ResetWheelStatesToZero.h"
#include "Commands/Autonomous/AutoCommands/ResetOdometryToASetPoint/ResetOdoToSetPoint.h"
#include "Commands/Autonomous/AutoCommands/GoToAngle/GoToAngle.h"
#include "Commands/Autonomous/AutoCommands/VisionAlign/VisionAlign.h"
#include "Commands/Shooter/DefaultIntake/UseIntakeOnTimer.h"
#include "Commands/Autonomous/AutoCommands/loadIntake/LoadIntake.h"


#include "Commands/Shooter/IdleShooter/SpinupShooter.h"
#include "Commands/Shooter/UseIntake/SetIntake.h"
#include "Commands/Shooter/Preload/PreloadBalls.h"
#include "Commands/Shooter/UseFeeder/FeedShooter.h"
#include "Commands/Shooter/ShootNBalls/ShootNBalls.h"
#include "Commands/Shooter/SetHood/SetHood.h"
#include "Commands/Shooter/UseShooter/DynamicSetShoot.h"


class VisionAlignLQRTest
        : public frc2::CommandHelper<frc2::SequentialCommandGroup, VisionAlignLQRTest> {
public:
    VisionAlignLQRTest(
                          const std::shared_ptr<EctoSwerve> &swerve,
                          VisionManager *visionManager);

private:
    std::unique_ptr<PathFollowerRotationProfile> firstPathRotationProfile;
    std::unique_ptr<PathFollowerRotationProfile> secondPathRotationProfile;
    std::unique_ptr<PathFollowerRotationProfile> thirdPathRotationProfile;
    std::unique_ptr<PathFollowerRotationProfile> fourthPathRotationProfile;
    std::unique_ptr<PathFollowerRotationProfile> getCloseToTerminalProfile;


};

#endif //BOTBUSTERS_REBIRTH_VISIONALIGNLQRTEST_H
