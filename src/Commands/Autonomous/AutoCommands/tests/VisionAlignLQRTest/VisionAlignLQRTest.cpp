//
// Created by cc on 13/04/22.
//

#include "VisionAlignLQRTest.h"


#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>
#include <wpi/fs.h>

#include "Commands/Autonomous/AutoCommands/IsOnOrNearTarget/IsOnTarget.h"
#include "Commands/Shooter/DefaultIntake/UseIntakeOnTimer.h"
#include "Commands/Shooter/UseFeeder/FeedShooter.h"
#include "Control/PathFollowers/PathFollowerRotationProfile.h"
#include "Commands/Shooter/DelayedBallShoot/DelayedBallShoot.h"
#include "Commands/Autonomous/AutoCommands/VisionAlignLQR/VisionAlignLQR.h"

VisionAlignLQRTest::VisionAlignLQRTest(
        const std::shared_ptr<EctoSwerve> &swerve,
        VisionManager *visionManager) {

    /**Base path follower config
     */


    AddCommands(
            VisionAlignLQR(swerve, visionManager)
    );
}