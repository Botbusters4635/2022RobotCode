//
// Created by cc on 25/07/22.
//

#ifndef BOTBUSTERS_REBIRTH_TOYOTAVIOLINPLAYINGROBOT_H
#define BOTBUSTERS_REBIRTH_TOYOTAVIOLINPLAYINGROBOT_H

#include "Core/EctoRobot.h"
#include "Core/VisionManager/VisionManager.h"
#include "Core/EctoInput/Buttons/EctoButton.h"

#include <frc2/command/Subsystem.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include "Utilities/WPI/SlewRateLimiter/RateLimiter.h"
#include <Core/PowerDistributionHub/PowerDistributionHub.h>

#include "Commands/Shooter/UseFeeder/FeedShooter.h"
#include "Commands/Shooter/UseIntake/SetIntake.h"
#include "Commands/Shooter/UseIndexer/UseIndexer.h"
#include "Commands/Shooter/IdleShooter/SpinupShooter.h"
#include "Commands/Shooter/UseShooter/DynamicSetShoot.h"
#include "Commands/Shooter/UseShooter/MultiPressureDynamicSetShoot.h"
#include "Commands/Shooter/DefaultIntake/UseIntakeOnTimer.h"

#include "Commands/Climber/UseClimber/UseClimber.h"
#include "Commands/Climber/HomeClimber/HomeClimber.h"
#include "Commands/Climber/StepByStepClimb/StepByStepClimb.h"
#include "Commands/Climber/StepByStepClimb/StepByStepClimb.h"

#include "Commands/Utilities/RumbleController/RumbleController.h"

#include "Control/PathFollowers/HolonomicPathFollower.h"

//Systems
#include <Systems/EctoSwerve/EctoSwerve.h>
#include <Systems/EctoSwerve/EctoSwerveInputHandler.h>
#include "Systems/LQRShooter/LQRShooter.h"
#include "Systems/Feeder/Feeder.h"
#include "Systems/PIDClimber/PIDClimber.h"
#include "Systems/Intake/Intake.h"
#include <Systems/PIDTurret/PIDTurret.h>

#include "Math/CalculateHoodAngle/CalculateHoodAngle.h"
#include "Math/CalculateShooterVel/CalculateShooterVel.h"
#include "Math/InterpolatingTable/InterpolatingTable.h"

#include "Commands/Utilities/StopwatchCommand/StopwatchCommand.h"
#include "Commands/Autonomous/AutoCommands/loadIntake/LoadIntake.h"

#include "Commands/LEDs/PIDShooterLEDs.h"

#include <frc/filter/SlewRateLimiter.h>
#include <frc/DutyCycle.h>
#include <frc/AnalogInput.h>
#include <frc/RobotState.h>
#include "Commands/Shooter/ShootAndMove/ShootAndMove.h"

//Simulation includes
#include "Systems/EctoSwerve/Simulation/EctoSwerveSim.h"
#include "Simulation/EctoFlywheelSim/EctoFlywheelSim.h"
#include "Simulation/EctoElevatorSim/EctoElevatorSim.h"

#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <Systems/GearBox/GearBox.h>
#include <Systems/PIDElevator/PIDElevator.h>
#include <PhotonLib/PhotonTrackedTarget.h>
#include <PhotonLib/PhotonCamera.h>
#include <PhotonLib/PhotonPipelineResult.h>

class ToyotaViolinPlayingRobot : public EctoRobot {
public:
    ToyotaViolinPlayingRobot();

    void disabledInit() override;

    void disabledUpdate() override;

    void robotInit() override;

    void robotUpdate() override;

    void autoInit() override;

    void autoUpdate() override;

    void teleopInit() override;

    void teleopUpdate() override;

    void simInit() override;

    void simUpdate() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {{EctoMotorType::SparkMax, "front_left_wheel",     7},
                {EctoMotorType::SparkMax, "front_right_wheel",    6},
                {EctoMotorType::SparkMax, "back_left_wheel",      1},
                {EctoMotorType::SparkMax, "back_right_wheel",     3},

                {EctoMotorType::SparkMax, "front_left_steer",     8},
                {EctoMotorType::SparkMax, "front_right_steer",    5},
                {EctoMotorType::SparkMax, "back_left_steer",      2},
                {EctoMotorType::SparkMax, "back_right_steer",     4},

                {EctoMotorType::SparkMax, "elevator",               12},
                {EctoMotorType::SparkMax, "elevatorFollower",       11},
                {EctoMotorType::SparkMax, "elevatorFollowerSecond", 10}
        };
    }
    std::list<PistonInfo> getPistonConfig(){
        return {
                {"gearBox", 20, frc::PneumaticsModuleType::REVPH, 0, 1}
        };
    }


private:

    enum class ControlBindingsState {
        Elevator,
        Climber
    };

    InputManager &input = InputManager::getInstance();
    PCMManager &pcm = PCMManager::getInstance();

    std::shared_ptr<EctoSwerve> swerve;
    std::shared_ptr<EctoSwerveInputHandler> swerveInputHandler;
    std::shared_ptr<VisionManager> visionManager;
//    std::shared_ptr<PowerDistributionHub> pdh;
//    std::shared_ptr<PIDClimber> climber;
    std::shared_ptr<PIDClimber> elevator;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;
    nt::NetworkTableEntry shooterVel, setHood, feederVel, currentPSI;

//    frc::SendableChooser<frc2::Command *> autoChooser;
//    frc2::Command *autoCommand{};

    std::shared_ptr<frc::Field2d> robotPose;
    std::shared_ptr<frc::Field2d> tagPose;
    std::shared_ptr<frc::Field2d> field2d;

    EctoButton
            climberMode,
            intakeMode,
            testSolenoid,

            homeHeight,
            topHeight,
            middleHeight,
            lowHeight,

            lowGoalManualShooter,
            tarmacManualShooter,
            visionAlignButton,
            alignTest;


    ShootAndMoveConfig shootAndMoveConfig;


    JoystickAxisExpo manualIntake{0.2, 0.2}, shootAndMove{0.2, 0.2};


    JoystickAxisExpo climberTrigger{0.2, 0.2};

    ToggleButton climberPistons, hookPistons;

    ControlBindingsState operatorControlMode{ControlBindingsState::Elevator};

    ControlBindingsState prevControlMode{ControlBindingsState::Climber};


    //Simulation
private:
    std::unique_ptr<EctoSwerveSim> swerveSim;
    std::unique_ptr<EctoFlywheelSim> flywheelSim;
    std::unique_ptr<EctoElevatorSim> elevatorSim;
    std::shared_ptr<frc::DoubleSolenoid> testPiston;
    photonlib::PhotonCamera camera{"camera"};//0

};


#endif //BOTBUSTERS_REBIRTH_TOYOTAVIOLINPLAYINGROBOT_H
