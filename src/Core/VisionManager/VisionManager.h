//
// Created by abiel on 2/12/20.
//

#ifndef BOTBUSTERSREBIRTH_VISIONMANAGER_H
#define BOTBUSTERSREBIRTH_VISIONMANAGER_H

#include "Core/EctoModule/WPISubsystem.h"
#include <Systems/EctoSwerve/EctoSwerve.h>
#include <Systems/PIDTurret/PIDTurret.h>
#include <frc/Timer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <optional>
#include <functional>
#include <frc/Notifier.h>
#include <frc/drive/Vector2d.h>
#include "frc/geometry/Velocity2d.h"
#include "VisionSource.h"
#include "Systems/LQRTurret/LQRTurret.h"
#include "TagPoses.h"
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc/geometry/Pose3d.h>

struct VisionManagerConfig{

    frc::Pose2d robotPose{{0_m, 0_m}, 0_rad};

    frc::Rotation2d robotYaw{0_rad};

    //TODO actually use these lol
    wpi::array<double, 3> statesStdDevs{0.0, 0.0, 0.0};
    wpi::array<double, 1> gyroEncoderStdDevs{0.0};
    wpi::array<double, 3> autoVisionStdDevs{0.0, 0.0, 0.0};
    wpi::array<double, 3> teleopVisionStdDevs{0.0, 0.0, 0.0};


    frc::Pose2d fieldToTarget{}; //Target position in field coord system
    units::meter_t targetHeight{0_m}; //Target height from floor

    std::vector<frc::Pose3d> cameraPoses;
    //Pitch is from the horizontal plane.
    //The rest is from the robot odometry zero


    aprilTags::TagPoses tagPoses;

};

class VisionManager : public WPISubsystem {
public:
    VisionManager(const std::shared_ptr<EctoSwerve> &swerve,
                  const std::shared_ptr<VisionSource> &source,
                  VisionManagerConfig &config);

    void autoInit() override;

    void teleopInit() override;

    void robotUpdate() override;

    //Returns estimated error to target (very experimental)
    double getYawError() const;

    double getTurretYawError() const;

    void setCameraToRobot(frc::Transform2d cameraToRobotPose){
        this->cameraToRobot = cameraToRobotPose;
    }

    frc::Rotation2d getRotationToTarget() const;

    frc::Pose2d getVisionPose() const;

    frc::Twist2d getVelocityToTarget() const;

    units::radians_per_second_t angularVelocityToTarget() const;

    //Use estimated measurements or vision measurements
    double getTargetDistance() const;

    bool hasValidTarget() const {
        return frc::Timer::GetFPGATimestamp() - targetTimestamp < 100_ms; //Invalid target if has not updated within 100ms
    }

    void ignoreVision(bool ignoreVision);

    void setLeds(bool state, int cameraIndex);

    units::radian_t calculateHeadingAdjustment(const frc::Translation2d &globalTurretTranslation, const frc::Velocity2d &drivetrainVelocity, units::radians_per_second_t flywheelVel) const;

    std::vector<std::shared_ptr<ChecklistItem>> createTests() override;
private:
    VisionManagerConfig config;

    std::shared_ptr<VisionSource> source;
    std::shared_ptr<EctoSwerve> swerve;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("VisionManager");
    std::shared_ptr<nt::NetworkTable> visionTable = ntInstance.GetTable("VisionManager/Vision");
    std::shared_ptr<nt::NetworkTable> tagTable = ntInstance.GetTable("aprilTags");


    units::second_t lastVisionRunTime;

    frc::Pose2d visionPose;

    units::second_t targetTimestamp{0};

    void updateVisionCircleFit(const VisionResult &result, const CameraInfo &cameraInfo);

    void updateVisionTelemetry(const VisionResult &result, const CameraInfo &cameraInfo);

    void updateTagVision(const TagResult &result, const CameraInfo &cameraInfo);

    std::unique_ptr<frc::Notifier> visionNotifier;

    VisionResult prevResult;

    TagResult prevTagRes;

    frc::Field2d visionPoseField;

    /**
     * Target velocity tracking
     */
    frc::Twist2d twistDelta;
    frc::Twist2d lastTwistToTarget;
    units::second_t lastTranslationToTargetTime;

    size_t processedFrames{0}, droppedFrames{0};

    template <typename Vector1, typename Vector2>
    static auto Cross(const Vector1& a, const Vector2& b) -> decltype(auto) {
        // (a_x i + a_y j) x (b_x i + b_y j)
        // = a_x b_y - a_y b_x
        return a.X() * b.Y() - a.Y() * b.X();
    }

    frc::Transform2d cameraToRobot;

    bool useVision{true};

    frc::LinearFilter<double> xFilter = frc::LinearFilter<double>::SinglePoleIIR(0.1, 5_ms);
    frc::LinearFilter<double> yFilter = frc::LinearFilter<double>::SinglePoleIIR(0.1, 5_ms);

    aprilTags::TagPoses tagPoses;

    frc::Pose2d tagPose;

    const bool filterVisionMeasurements = true;



    std::shared_ptr<frc::Field2d> tag0Pose;
    std::shared_ptr<frc::Field2d> tag3Pose;
    std::shared_ptr<frc::Field2d> tag27Pose;

//    frc::Transform2d cameraPose{};

//    frc::KalmanFilter<3, 3, 3> tagPoseObserver;
//    frc::LinearSystem<3,3,1> tagPosePlant;
//    frc::LinearFilter<double> thetaFilter = frc::LinearFilter<double>::SinglePoleIIR(0.0115, 5ms);

};

#endif//BOTBUSTERSREBIRTH_VISIONMANAGER_H
