//
// Created by cc on 25/07/22.
//

#include "ToyotaViolinPlayingRobot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Core/VisionManager/LimelightSource.h"
#include "Commands/Utilities/SwerveDistanceAlign/SwerveDistanceAlign.h"
#include <frc2/command/PerpetualCommand.h>
#include "Commands/Shooter/DelayedBallShoot/DelayedBallShoot.h"
#include "Commands/Autonomous/AutoCommands/MoveToPoint/MoveToPoint.h"
#include <units/acceleration.h>
#include "Commands/Climber/SetClimber/ClimberInputHandler.h"
#include "Commands/Turret/TurretInputHandler/TurretInputHandler.h"
#include <units/current.h>
#include "Control/TrajectoryGenerator.h"
#include <Core/VisionManager/PhotonvisionSource.h>

ToyotaViolinPlayingRobot::ToyotaViolinPlayingRobot() : EctoRobot("ToyotaViolinPlayingRobot") {
    table = ntInstance.GetTable("1-ToyotaViolinPlayingRobot");
    shooterVel = table->GetEntry("ShooterVel");
    setHood = table->GetEntry("SetHood");
    feederVel = table->GetEntry("FeederVel");
    currentPSI = table->GetEntry("currentPSI");

}

void ToyotaViolinPlayingRobot::disabledInit() {
    ;
}

void ToyotaViolinPlayingRobot::disabledUpdate() {
    ;
}

void ToyotaViolinPlayingRobot::robotInit() {
    table->GetEntry("setLeds").SetBoolean(false);
    EctoSwerveConfig swerveConfig;
    swerveConfig.length = 0.8636; //meters 34 in
    swerveConfig.width = 0.866775; // meters 34.125 in
    swerveConfig.wheelCircumference = 0.0508 * 2 * M_PI;
    swerveConfig.gearRatio = 6.92;

    VisionManagerConfig visionConfig;
    visionConfig.statesStdDevs = wpi::array<double, 3>{0.01, 0.01, 0.01};
    visionConfig.gyroEncoderStdDevs = wpi::array<double, 1>{0.1};
    visionConfig.autoVisionStdDevs = wpi::array<double, 3>{0.1, 0.1, 0.1};
    visionConfig.teleopVisionStdDevs = wpi::array<double, 3>{0.4, 0.4, 0.4};
    visionConfig.robotPose = frc::Pose2d{{0_m, 0_m}, 0_rad};
    visionConfig.robotYaw = frc::Rotation2d{0_rad};

    visionConfig.tagPoses.tag16h5 = {
            {2, {8.9_m, 0.635_m, {0_deg}}},
            {3, {8.63_m, 6.755_m, {0_deg}}},
            {27, {5.93_m, 0.145_m, {0_deg}}}//added 50cm to x

    };


//    visionConfig.cameraToRobot = frc::Transform2d(frc::Translation2d(-0.3506_m, -0.2496_m),
//                                                  frc::Rotation2d(180_deg));
    visionConfig.targetHeight = units::centimeter_t(263);
    visionConfig.fieldToTarget = frc::Pose2d(8.25_m, 4.07_m, {0_deg});

    visionConfig.cameraPoses = {
            frc::Pose3d(-5.638_in, 7.46787_in, 31.1084_in + 1.5_in, {-90_deg, 0_deg, 90_deg}),
            frc::Pose3d(-5.638_in, -7.46787_in, 31.1084_in + 1.5_in, {90_deg, 0_deg, -90_deg}),




    };

//    GearBoxConfig gearBoxConfig;
//    gearBoxConfig.shifters = {pcm.getPiston("gearBox")}

    PIDClimberConfig elevatorConfig;
    elevatorConfig.pidConfig.p = 48.101;
    elevatorConfig.pidConfig.i = 0;
    elevatorConfig.pidConfig.d = 3.93145;
    elevatorConfig.pidConfig.f = 0;
    elevatorConfig.motors = {
            motorManager.getMotor("elevator"),
            motorManager.getMotor("elevatorFollower"),
            motorManager.getMotor("elevatorFollowerSecond")
    };
    elevatorConfig.isInverted = {true, false, false};
    elevatorConfig.gearReduction = 5.2525252525;
    elevatorConfig.currentLimit = 35;
    elevatorConfig.rampRate = 0.01;
    elevatorConfig.freeFF = {0.12414_V, 0.043476_V, 4.7943_V / 1_mps, 0.17883_V / 1_mps_sq};
    elevatorConfig.loadedFF = {0.24319_V, 0.15664_V, 4.5228_V / 1_mps, 0.24778_V / 1_mps_sq};
    elevatorConfig.pulleyDiameter = 0.04826;
    elevatorConfig.enableLimitSwitch = true;
    elevatorConfig.forwardSoftLimit = 1.6;
    elevatorConfig.reverseSoftLimit = 0.0;
    elevatorConfig.maxVelocity = 8;
    elevatorConfig.maxAcceleration = 3.5;
    elevatorConfig.pidTolerance = 0.001;


    swerve = std::make_shared<EctoSwerve>(swerveConfig);
//    pdh = std::make_shared<PowerDistributionHub>(1, frc::PowerDistribution::ModuleType::kRev);
    auto visionSource = std::make_shared<PhotonvisionSource> (swerve);
    visionSource->setLeds(false, 1);
    visionSource->setLeds(false, 0);
    visionManager = std::make_shared<VisionManager>(swerve, visionSource, visionConfig);

    elevator = std::make_shared<PIDClimber>(elevatorConfig);

    robotPose = std::make_shared<frc::Field2d>();

//    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

    //Mechanism Controller
    input.registerButton(climberMode, "start", 0);
    input.registerButton(intakeMode, "leftJoystick", 0);

    input.registerButton(lowGoalManualShooter, "leftJoystick", 1); //B de bigg brain
    input.registerButton(tarmacManualShooter, "rightJoystick", 1);

    input.registerButton(testSolenoid, "rightBumper", 1);

    input.registerButton(homeHeight, "X", 1);
    input.registerButton(topHeight, "Y", 1);
    input.registerButton(middleHeight, "B", 1);
    input.registerButton(lowHeight, "A", 1);

    input.registerButton(alignTest, "A", 0);

    swerve->SetDefaultCommand(EctoSwerveInputHandler(swerve, visionManager.get()));

    frc::SmartDashboard::PutData("RobotPose", robotPose.get());
    frc::SmartDashboard::PutBoolean("gearBox", false);
    frc::SmartDashboard::PutNumber("elevator/pose", 0);
    testPiston = pcm.getPiston("gearBox");
    elevator->resetToZero();

    tagPose = std::make_shared<frc::Field2d>();
    frc::SmartDashboard::PutData("estimatedTagPose", tagPose.get());

//    frc::TrajectoryConfig trajConfig(units::meters_per_second_t(2),
//                                 units::meters_per_second_squared_t(1));
//
//
//    auto firstTraj = botbusters::TrajectoryGenerator::noHeadingTrajectory({
//                                                                                  {5_m, 4.5_m},
//                                                                                  {6_m, 5.324_m},
//                                                                                  {7_m, 6.45_m}
//
//    }, trajConfig);
//    field2d = std::make_shared<frc::Field2d>();
//    field2d->GetObject("testTraj")->SetTrajectory(firstTraj);
//    frc::SmartDashboard::PutData("traj", field2d.get());

}

void ToyotaViolinPlayingRobot::robotUpdate() {
    PCMManager::set(testPiston, frc::SmartDashboard::GetBoolean("gearBox", false));
    robotPose->SetRobotPose(swerve->getPose());
    table->GetEntry("pressure").SetDouble(pcm.getAnalogPressure().value());

    frc::SmartDashboard::PutNumber("tiltX", swerve->getRoll());
    frc::SmartDashboard::PutNumber("tiltY", swerve->getPitch());
    frc::SmartDashboard::PutNumber("tiltXdeg", EctoMath::radiansToDegrees(swerve->getRoll()));
    frc::SmartDashboard::PutNumber("tiltYdeg", EctoMath::radiansToDegrees(swerve->getPitch()));



//
//
//
//    units::radian_t cameraOffset{-90_deg};
//    frc::Rotation2d cameraYaw = units::radian_t (swerve->getYaw()) + cameraOffset;
//
//    frc::Translation2d aprilTagPose{5.43_m, 0.145_m};
//
//
//
//    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
//
//    photonlib::PhotonTrackedTarget target = result.GetBestTarget();
//    frc::Transform3d cameraToTarget = target.GetBestCameraToTarget();
//
////    targetID = target.GetCameraRelativePose();
//
//
//    auto x = units::meter_t(frc::SmartDashboard::GetNumber("test/x", 0));
//    auto y = units::meter_t(frc::SmartDashboard::GetNumber("test/y", 0));
//
//
//    frc::Translation2d camera2d{cameraToTarget.X(), cameraToTarget.Y()};
//    frc::Translation2d robot2d = camera2d.RotateBy(cameraYaw);
//
//    frc::Translation2d robot{(aprilTagPose.X() - robot2d.X()) + -5.619462_in, (aprilTagPose.Y() - robot2d.Y()) + -7.436787_in};
//

    tagPose->SetRobotPose(visionManager->getVisionPose());


//    table->GetEntry("analogPressure").SetDouble(PCMManager::getAnalogPressure().value());

    visionManager->setLeds(table->GetEntry("setLeds").GetBoolean(false), 0);
    visionManager->setLeds(table->GetEntry("setLeds").GetBoolean(false), 1);

//    ledsTable->GetEntry(isBlueAllianceEntryName).SetBoolean(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue);
}

void ToyotaViolinPlayingRobot::autoInit() {
//    visionManager->setLeds(true);
//    autoCommand = autoChooser.GetSelected();
//    commandScheduler.Schedule(autoCommand);

}

void ToyotaViolinPlayingRobot::autoUpdate() {
    ;
}

void ToyotaViolinPlayingRobot::teleopInit() {
    swerve->resetOdometry({5.17_m, 4.132_m, {0_deg}});
    elevator->setFF(PIDClimber::FeedForward::Free);
    elevator->set(0);

    homeHeight.WhenPressed([elevator = elevator] {
                               elevator->set(0);
                           }, {elevator.get()}
    );
    topHeight.WhenPressed([elevator = elevator] {
                               elevator->set(1.6);
                           }, {elevator.get()}
    );
    middleHeight.WhenPressed([elevator = elevator] {
                               elevator->set(1.1);
                           }, {elevator.get()}
    );
    lowHeight.WhenPressed([elevator = elevator] {
                               elevator->set(0.5);
                           }, {elevator.get()}
    );

    frc::SmartDashboard::PutNumber("test/x", 0);
    frc::SmartDashboard::PutNumber("test/y", 0);

    MoveToPointConfig pointConfig;
    pointConfig.pos_p = 5.0;
    pointConfig.pos_i = 0.0;
    pointConfig.pos_d = 0.0;
    pointConfig.theta_p = 7.5;
    pointConfig.theta_i = 0.0;
    pointConfig.theta_d = 0.0;
    pointConfig.maxVel = 1.5_mps;
    pointConfig.maxAccel = 1_mps_sq;
    pointConfig.maxAngularVel = 3_rad_per_s;
    pointConfig.maxAngularAccel = 3_rad_per_s_sq;

    alignTest.WhileHeld(new MoveToPoint(
            swerve,
            visionManager,
            frc::Pose2d(6.8_m, 4.25_m, {0_deg}),
            pointConfig
    ));




////    intake->set(1, true);
//    visionManager->ignoreVision(false);
//    frc::SmartDashboard::PutNumber("turretSetpoint", 0);
//    feeder->setFeederVol(12);
//    turret->SetDefaultCommand(ConstantAlign(turret, visionManager.get(), false));

}

void ToyotaViolinPlayingRobot::teleopUpdate() {

//    double maxTilt = std::max(std::abs(swerve->getRoll()), std::abs(swerve->getPitch()));
//    frc::SmartDashboard::PutNumber("maxTilt", maxTilt);
//    bool elevatorDown = maxTilt > 0.1745329;
//    frc::SmartDashboard::PutBoolean("elevatorDown", elevatorDown);
//    if(elevatorDown){
//        elevator->set(0);
//    }


//    frc::SmartDashboard::PutNumber("test/inverse/x", testTranslation.Inverse.X().value());
//    frc::SmartDashboard::PutNumber("test/inverse/y", testTranslation.Y().value());

//    gearBox->engage(frc::SmartDashboard::GetBoolean("gearBox", false));

//    if (intakeMode.get()) {
//        operatorControlMode = ControlBindingsState::IntakeShooterFeeder;
//    }
//    if (climberMode.get()) {
//        operatorControlMode = ControlBindingsState::Climber;
//    }
//
//
//    if (operatorControlMode != prevControlMode) {
//        if (operatorControlMode == ControlBindingsState::IntakeShooterFeeder) {
//            commandScheduler.CancelAll();
//            commandScheduler.ClearButtons();
//
//            visionManager->setLeds(true);
//        }
//        if (operatorControlMode == ControlBindingsState::Climber) {
//            commandScheduler.CancelAll();
//            commandScheduler.ClearButtons();
////
//////            homeClimber.WhenPressed(HomeClimber(climber));
////            startClimberSequence.WhenPressed(stepByStepClimb.get());
////            stopClimberSequence.WhenReleased([this] { commandScheduler.Cancel(stepByStepClimb.get()); });
////            intakeButton.WhileHeld(SetIntake(intake, 0.0, true)).WhenReleased(SetIntake(intake, 0.0, false));
//            visionManager->setLeds(false);
//        }
//        prevControlMode = operatorControlMode;
//    }

}

void ToyotaViolinPlayingRobot::simInit() {
    log->info("This should not be running");
    swerveSim = std::make_unique<EctoSwerveSim>(swerve);
    managerHandler.addManager(*swerveSim);

    swerveSim->resetPose({7.3298_m, 1.7478_m, -91.5_deg});

//    flywheelSim = std::make_unique<EctoFlywheelSim>(
//            motorManager.getSimulatedMotor("shooterMotor"),
//            frc::DCMotor::NEO(2),
//            1,
//            0.019842, 0.0022638);

//    managerHandler.addManager(*flywheelSim);

//    elevatorSim = std::make_unique<EctoElevatorSim>(
//            motorManager.getSimulatedMotor("frontClimber"),
//            frc::DCMotor::NEO(2),
//            10_kg,
//            0.04064_m,
//            20.94
//    );

//    managerHandler.addManager(*elevatorSim);
}

void ToyotaViolinPlayingRobot::simUpdate() {

}
