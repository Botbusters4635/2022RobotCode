//
// Created by abiel on 1/2/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOSWERVE_H
#define BOTBUSTERSREBIRTH_ECTOSWERVE_H

#include <Core/EctoModule/WPISubsystem.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <Core/EctoInput/Buttons/EctoButton.h>

#include <Control/SimpleControllerSource.h>
#include <Control/SimpleControllerOutput.h>

#include <Control/Kinematics/Swerve/SwerveKinematics.h>
#include <Control/Kinematics/Swerve/SwerveState.h>

#include <Control/Odometry/LinearOdometry.h>
#include <Control/Odometry/ExponentialOdometry.h>

#include "Core/EctoInput/InputManager.h"
#include <Core/MotorHandler/MotorManager.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/smartdashboard/Field2d.h"
#include <frc/DigitalOutput.h>
#include "Sensors/EctoDistanceSensor.h"

#include "Math/EctoMath.h"

#include "Control/EctoPID/PIDConfig.h"

#include <frc/ADIS16470_IMU.h>

#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Notifier.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include "frc/manual_merge/SwerveDrivePoseEstimator.h"
#include <frc/geometry/Pose2d.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "GenericSwerveValue.h"

#include <functional>

#include "Control/Kinematics/Swerve/SwerveKinematics.h"

#include "SwerveModule/SwerveModule.h"
#include "Utilities/NetworkTablePID/NetworkTablePID.h"

#include <frc/controller/SimpleMotorFeedforward.h>

#include "frc/geometry/Velocity2d.h"
#include <frc/filter/MedianFilter.h>

#define USE_NAVX
#ifdef USE_NAVX
#include <AHRS.h>
#endif


struct EctoSwerveConfig {
    //Angles to which the robot will snap to
    std::vector<double> snappableAngles{0, 2.69, M_PI / 2.0, 0.58, M_PI, -0.58, -M_PI / 2.0, -2.72};

    double gearRatio = 8.33;
    double wheelCircumference = 0.0508 * M_PI;
    double length = 1.0;
    double width = 1.0;


};

class SwerveThetaController;

class EctoSwerve : public WPISubsystem {
public:
    explicit EctoSwerve(const EctoSwerveConfig &config);

    void robotInit() override;

    void robotUpdate() override;

    void zeroYaw();

    void resetOdometry(const frc::Pose2d newPose = {0_m, 0_m, {0_deg}});

    void setYaw(double yaw);

    double getYaw(bool useZero = true) const;

    double getRoll(bool useZero = true) const;

    double getPitch(bool useZero = true) const;

    double getRawYaw() const;

    double simYaw{0};

    double getYawRate() const {
        double rate = 0;

#ifndef SIMULATION
#ifndef USE_NAVX
        rate = (adis->GetRate().value() / 180.0) * M_PI;
#else
        //TODO maybe rate is negative
        rate = -navx->GetRate() * (M_PI / 180.0);
#endif
#endif

        return rate;
    }

    void setVelocity(const frc::ChassisSpeeds &target);

    void setPercent(const frc::ChassisSpeeds &target, const Point2D &point = Point2D(0, 0));

    void setVoltage(const frc::ChassisSpeeds &target);

    void setModules(const SwerveState &setpoint, MotorControlMode controlMode);

    void setModules(const std::array<frc::SwerveModuleState, 4> &state, bool velocityControl = true) {
        setModules(SwerveState(state), velocityControl);
    }

    void setModules(const SwerveState &rawSetpoint, bool velocityControl = true) {
        setModules(rawSetpoint, velocityControl ? MotorControlMode::Velocity : MotorControlMode::Percent);
    }

    frc::Pose2d getPose();

    void addVisionPoseMeasurement(const frc::Pose2d visionPose, double timeStamp);

    SwerveState getMotorStates() const;

    frc::Velocity2d getVelocity() const;

    frc::Rotation2d getRotation();

    frc::ChassisSpeeds getChassisSpeeds();

    std::shared_ptr<frc::SwerveDriveKinematics<4>> getKinematics() const { return kinematics; }

    std::vector<std::shared_ptr<ChecklistItem>> createTests() override;

    units::volt_t calculateRotationFF(const units::radians_per_second_t &velocity) const;

    void setVisionStdDev(const wpi::array<double, 3> &stdDevs){
        pe_NMutex.lock();
        pe_MMutex.lock();
        pe_NMutex.unlock();
        odometry->SetVisionMeasurementStdDevs(stdDevs);
        pe_MMutex.unlock();
    }

    std::shared_ptr<SwerveThetaController> getThetaController() const {
        return thetaController;
    }

    double getVelMagitude() const {
        return velMagnitude;
    }
private:
    void initNetworkTables();

    void updateNetworkTables();

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    EctoSwerveConfig config;
    double velMagnitude{0};

    //TODO Implement odometry for setTargetPosition
    /**
     * Kinematics
     */
    std::shared_ptr<frc::SwerveDriveKinematics<4>> kinematics;

    static std::shared_ptr<frc::SwerveDriveKinematics<4>>
    createKinematics(units::meter_t length, units::meter_t width) {
        frc::Translation2d frontLeft{width, length};
        frc::Translation2d frontRight{width, -length};
        frc::Translation2d backLeft{-width, length};
        frc::Translation2d backRight{-width, -length};

        return std::make_shared<frc::SwerveDriveKinematics<4>>(frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * PIDS
     */
    PIDConfig headingPIDConfig;
    PIDConfig wheelVelocityPID;

    std::array<std::unique_ptr<SwerveModule>, 4> modules;
//	std::unique_ptr<NetworkTablePID> steerPIDNT, wheelPIDNT;

    /**
     * Motors
     */
    MotorManager &motorHandler = MotorManager::getInstance();

    /**
     * Heading
     */
    double headingZero = 0.0;
    double rollZero = 0.0;
    double pitchZero = 0.0;



#ifndef SIMULATION

#ifndef USE_NAVX
    std::unique_ptr<frc::ADIS16470_IMU> adis{};
#else
    std::unique_ptr<AHRS> navx;
#endif

#endif

    void updateOdometry(const SwerveState &motorStates);

    /**
     * Odometry
     */
    std::unique_ptr<botbusters::SwerveDrivePoseEstimator<4>> odometry;
    frc::ChassisSpeeds currentVelocity{};

    mutable std::mutex pe_MMutex, pe_NMutex, pe_LMutex; //https://stackoverflow.com/questions/11666610/how-to-give-priority-to-privileged-thread-in-mutex-locking

    frc::SimpleMotorFeedforward<units::radian> rotationFF {0.25438_V, 1.7431_V / 1_rad_per_s, 0.23028_V / 1_rad_per_s_sq};

    /**
     * Kinematics optimization
     */
    SwerveState lastState, lastSetpoint;
    double lastRunTime = 0;

    const bool poseido = false;

    std::shared_ptr<SwerveThetaController> thetaController;
    units::second_t lastSetTime = 0_s;
    bool lastThetaControllerState = false;

    frc::LinearFilter<double> xFilter = frc::LinearFilter<double>::SinglePoleIIR(0.0119, 20_ms);
    frc::LinearFilter<double> yFilter = frc::LinearFilter<double>::SinglePoleIIR(0.0119, 20_ms);


};

/**
 * Dummy (thicc) wpi system so angular velocity can be controlled
 * from a command while keeping teleop control, note, theta is in volts
 * Example:
 * BasicCommand::BasicCommand(){
 *      ...
 *      AddRequirements({swerve->getThetaController});
 *      //tada
 */
#include <iostream>
class SwerveThetaController : public frc2::SubsystemBase {
public:
    SwerveThetaController(){;}

    void setTheta(units::volt_t target){
        this->target = target;
    }

    [[nodiscard]] units::volt_t getTarget() const {
        return target;
    }

    void enable(bool state){
        enabled = state;
    }

    bool isEnabled() const{
        return enabled;
    }
private:
    friend class EctoSwerve;

    units::volt_t target{0_V};
    bool enabled{false};
};


#endif //BOTBUSTERSREBIRTH_ECTOSWERVE_H
