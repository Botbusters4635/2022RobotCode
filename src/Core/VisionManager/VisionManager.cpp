//
// Created by abiel on 2/12/20.
//

#include "VisionManager.h"
#include "frc/StateSpaceUtil.h"
#include <Systems/EctoSwerve/EctoSwerve.h>
#include "Math/VisionUtilities.h"
#include <cmath>
#include "Utilities/ChecklistTests/SSHConnectionTest.h"

VisionManager::VisionManager(const std::shared_ptr<EctoSwerve> &swerve,
                             const std::shared_ptr<VisionSource> &source,
                             VisionManagerConfig &config) :
                             WPISubsystem("VisionManager"){
    this->config = config;
    this->swerve = swerve;
    this->source = source;
    this->tagPoses = config.tagPoses;

//    this->cameraToRobot = config.cameraToRobot;
    tag0Pose = std::make_shared<frc::Field2d>();
    tag3Pose = std::make_shared<frc::Field2d>();
    tag27Pose = std::make_shared<frc::Field2d>();
    frc::SmartDashboard::PutData("estimatedTagPose/0", tag0Pose.get());
    frc::SmartDashboard::PutData("estimatedTagPose/3", tag3Pose.get());
    frc::SmartDashboard::PutData("estimatedTagPose/27", tag27Pose.get());

    visionNotifier = std::make_unique<frc::Notifier>([&] {
        auto dt = frc::Timer::GetFPGATimestamp() - lastVisionRunTime;
        auto result = this->source->getLatestResult();
        auto cameraInfo = this->source->getCameraInfo();
        auto tagRes = this->source->getLatestTagResult();

        if(tagRes.timestamp != prevTagRes.timestamp){
            updateTagVision(tagRes, cameraInfo);
            prevTagRes = tagRes;
        }

        if (result.timestamp != prevResult.timestamp) {
            //Update vision result

            updateVisionCircleFit(result, cameraInfo);
            updateVisionTelemetry(result, cameraInfo);
            visionTable->GetEntry("dt").SetDouble(dt.value());
            prevResult = result;
        };
        lastVisionRunTime = frc::Timer::GetFPGATimestamp();
    });

#ifndef SIMULATION
    visionNotifier->StartPeriodic(units::millisecond_t(5));
#endif
}

void VisionManager::autoInit() {

}

void VisionManager::teleopInit() {

}



double VisionManager::getTargetDistance() const {
    return swerve->getPose().Translation().Distance(config.fieldToTarget.Translation()).value();
}

frc::Rotation2d VisionManager::getRotationToTarget() const{
    double yawError = getYawError();
    auto rot = swerve->getRotation();
    return rot.RotateBy(frc::Rotation2d(units::radian_t(-yawError)));
}

double VisionManager::getYawError() const {
    //Estimates yaw error to target based on fused measurements,
    //Asumes target is round
    frc::Pose2d pose = swerve->getPose();
    double errorAngle = std::atan2(
            (config.fieldToTarget.Y() - pose.Y()).value(),
            (config.fieldToTarget.X() - pose.X()).value());
    errorAngle -= pose.Rotation().RotateBy(frc::Rotation2d(units::degree_t(-45))).Radians().value(); //Might need to be offset by 90 deg or something, depending on cs
    return EctoMath::wrapAngle(-errorAngle);
}

void VisionManager::ignoreVision(bool ignoreVision) {
    useVision = !ignoreVision;
}

double VisionManager::getTurretYawError() const {
    return -getYawError();

}

void VisionManager::updateTagVision(const TagResult &result, const CameraInfo &cameraInfo) {
    if (!result.hasTargets || result.tags.empty() || result.latency > 450_ms){ return;}
    auto robotYaw = units::radian_t(swerve->getYaw());

    std::vector<int> tagIds{};
    std::vector<int> cameraIds{};

    for (auto &tag : result.tags){

        robotYaw = units::radian_t(swerve->getYaw());

        auto camPose = config.cameraPoses[tag.cameraId];
        auto cameraOffset = frc::Rotation2d(robotYaw + camPose.Rotation().Z());
        frc::Transform2d cameraPose = {camPose.Translation().ToTranslation2d(), cameraOffset};

        frc::Translation2d fieldCentric = VisionUtilities::calculateFieldCentricFromTag(
                tag.tagPose,
                cameraPose,
                tagPoses.tag16h5.find(tag.id)->second
                );

        tagIds.emplace_back(tag.id);
        cameraIds.emplace_back(tag.cameraId);
        auto filteredX = units::meter_t(xFilter.Calculate(fieldCentric.X().value()));
        auto filteredY = units::meter_t(yFilter.Calculate(fieldCentric.Y().value()));

        tagPose = frc::Pose2d(filteredX, filteredY, frc::Rotation2d(robotYaw));
        visionPose = tagPose;
        //make vision filter in vision utilites and filetr respons times plus other things
        bool res = false;
        if(filterVisionMeasurements){
            res = VisionUtilities::filterTagPose(tag, swerve);
        }

        if (!res){
            return;
        }

        if(tag.id == 27){
            tag27Pose->SetRobotPose(tagPose);
        }
        if(tag.id == 3){
            tag3Pose->SetRobotPose(tagPose);
        }
        if(tag.id == 0){
            tag0Pose->SetRobotPose(tagPose);
        }
        swerve->addVisionPoseMeasurement(tagPose, tag.timestamp.value());
    }
    VisionUtilities::publishVector(tagTable->GetEntry("tagIds"), tagIds);
    VisionUtilities::publishVector(tagTable->GetEntry("cameraIds"), cameraIds);


}

void VisionManager::robotUpdate() {

    auto dt = frc::Timer::GetFPGATimestamp() - lastTranslationToTargetTime;
    frc::Pose2d poseRelative = swerve->getPose().RelativeTo(config.fieldToTarget);
    frc::Twist2d twistToTarget;
    twistToTarget.dx = poseRelative.X();
    twistToTarget.dy = poseRelative.Y();
    twistToTarget.dtheta = poseRelative.Rotation().Radians();

    twistDelta = twistToTarget;
    twistDelta.dx -= lastTwistToTarget.dx;
    twistDelta.dy -= lastTwistToTarget.dy;
    twistDelta.dtheta -= lastTwistToTarget.dtheta;
    twistDelta = twistDelta * (1.0/dt.value());

    lastTwistToTarget = twistToTarget;
    lastTranslationToTargetTime = frc::Timer::GetFPGATimestamp();

    table->GetEntry("TargetPos/X").SetDouble(twistDelta.dx.value());
    table->GetEntry("TargetPos/Y").SetDouble(twistDelta.dy.value());
    table->GetEntry("ProcessedFrames").SetDouble(processedFrames);
    table->GetEntry("DroppedFrames").SetDouble(droppedFrames);
    table->GetEntry("YawError").SetDouble(getYawError());
    if(processedFrames != 0 && droppedFrames != 0){
        table->GetEntry("PctDropped").SetDouble((float)droppedFrames/ (float) processedFrames);
    }
}

frc::Pose2d VisionManager::getVisionPose() const {
    return visionPose;
}

void VisionManager::updateVisionCircleFit(const VisionResult &result, const CameraInfo &cameraInfo) {
    return;
    if (!result.hasTargets || result.targets.size() < 2 || !result.ledState) return;

    targetTimestamp = result.timestamp;

    std::vector<frc::Translation2d> outputTranslations;
    std::vector<std::string> sortedPoints;

    for(const auto &target : result.targets){
        std::vector<std::pair<double, double>> targetPoints, topTargets, bottomTargets;

        targetPoints = target.contourPoints;
        processedFrames++;

        if(targetPoints.size() % 4 != 0){
            //Drop frame
            droppedFrames++;
            continue;
        }

        //Sort rectangle points
        targetPoints = VisionUtilities::sortRectanglePoints(targetPoints);
        std::vector<std::string> dirs = {"tl","tr","bl","br"};
        for(size_t i = 0; i < 4; i++){
            sortedPoints.emplace_back(fmt::format("{},{},{}",dirs[i],targetPoints[i].first, targetPoints[i].second));
        }

        if(targetPoints.size() != 4) continue;
        std::copy_n(targetPoints.begin(), 2, std::back_inserter(topTargets)); //First 2 targets
        std::copy_n(targetPoints.rbegin(), 2, std::back_inserter(bottomTargets)); //Last two points


        //Calculate output translations for each point
        std::for_each(topTargets.begin(), topTargets.end(), [&](const auto &corner){
            auto ret = VisionUtilities::cameraToTargetTranslation(corner,
                                                                  cameraInfo.resolution,
                                                                  2.0 * units::math::tan(cameraInfo.horizontalFov / 2.0),
                                                                  2.0 * units::math::tan(cameraInfo.verticalFov / 2.0),
                                                                  10.0,
                                                                  10.0,
                                                                  config.targetHeight.value());
            if(ret.has_value()) outputTranslations.push_back(std::move(ret.value()));
        });

        std::for_each(bottomTargets.rbegin(), bottomTargets.rend(), [&](const auto &corner){
            auto ret = VisionUtilities::cameraToTargetTranslation(corner,
                                                                  cameraInfo.resolution,
                                                                  2.0 * units::math::tan(cameraInfo.horizontalFov / 2.0),
                                                                  2.0 * units::math::tan(cameraInfo.verticalFov / 2.0),
                                                                  10.0,
                                                                  10.0,
                                                                  config.targetHeight.value() - (0.0254 * 2));
            if(ret.has_value()) outputTranslations.push_back(std::move(ret.value()));
        });
    }

    //Now the interesting part, fit a circle to all targets
    std::vector<std::pair<double,double>> circlePoints;
    std::transform(outputTranslations.begin(), outputTranslations.end(), std::back_inserter(circlePoints),
                   [](const auto &translation) {
                       return std::pair(translation.X().value(), translation.Y().value());
                   });


    auto circle = VisionUtilities::solveLeastSquaresCircle(circlePoints);
    if(!circle.has_value()) {
        log->warn("Could not fit circle");
        return; //Oh no
    }

    VisionUtilities::Circle circleValue = circle.value();
    const std::pair<double, double> circleMidpoint = circleValue.midpoint;


    VisionUtilities::publishCircle(table->GetEntry("VisionCircle"), table->GetEntry("VisionMidpoint"), circleValue);

    if(std::abs(circleValue.radius - 0.46) > 0.2) return;

    frc::Translation2d circleTranslation {units::meter_t(circleMidpoint.first), units::meter_t(circleMidpoint.second)};
    frc::Rotation2d robotRotation = swerve->getPose().Rotation();
    frc::Rotation2d cameraRotation = robotRotation.RotateBy(cameraToRobot.Rotation());
    frc::Transform2d fieldToTargetRotated {config.fieldToTarget.Translation(), cameraRotation};

    frc::Transform2d fieldToCamera = fieldToTargetRotated + frc::Transform2d(-circleTranslation, {});
    frc::Transform2d fieldToRobot = fieldToCamera + cameraToRobot.Inverse();
    visionPose = {fieldToRobot.Translation(), fieldToRobot.Rotation()};
//    visionPose = {fieldToRobot.Translation(), units::radian_t(thetaFilter.Calculate(fieldToRobot.Rotation().Radians().value()))};
//    visionPose = {{units::meter_t(xFilter.Calculate(fieldToRobot.X().value())), units::meter_t(yFilter.Calculate(fieldToRobot.Y().value()))}, fieldToRobot.Rotation()};
    if (useVision){
        swerve->addVisionPoseMeasurement(visionPose, targetTimestamp.value());
    }
    visionPoseField.SetRobotPose(visionPose);
    frc::SmartDashboard::PutData("Limelight/VisionPoseField", &visionPoseField);
    VisionUtilities::publishPoint(visionTable->GetEntry("EstimatedVisionPose"), {visionPose.X().value(), visionPose.Y().value()});

    table->GetEntry("SortedPoints").SetStringArray(sortedPoints);
}

void VisionManager::updateVisionTelemetry(const VisionResult &result, const CameraInfo &cameraInfo) {
    visionTable->GetEntry("HasTargets").SetBoolean(result.hasTargets);
    visionTable->GetEntry("IsValid").SetBoolean(hasValidTarget());
    if (result.hasTargets) {
        visionTable->GetEntry("TargetCount").SetDouble(result.targets.size());
    }

    visionTable->GetEntry("VisionPose/X").SetDouble(visionPose.X().value());
    visionTable->GetEntry("VisionPose/Y").SetDouble(visionPose.Y().value());
    visionTable->GetEntry("VisionPose/Point").SetString(
            fmt::format("({},{})", visionPose.X().value(), visionPose.Y().value()));
    visionTable->GetEntry("VisionPose/theta").SetDouble(visionPose.Rotation().Radians().value());

    visionTable->GetEntry("Latency").SetDouble(result.latency.value() / 1000.0);
    visionTable->GetEntry("YawError").SetDouble(getYawError());
    visionTable->GetEntry("turretYawError").SetDouble(getTurretYawError());
    visionTable->GetEntry("Distance").SetDouble(getTargetDistance());
    visionTable->GetEntry("VelocityToTarget/X").SetDouble(getVelocityToTarget().dx.value());
    visionTable->GetEntry("VelocityToTarget/Y").SetDouble(getVelocityToTarget().dy.value());
    visionTable->GetEntry("AngularVelToTarget").SetDouble(angularVelocityToTarget().value());
    visionTable->GetEntry("useVision").SetBoolean(useVision);

    tagTable->GetEntry("tagDebug/tagPose/x").SetDouble(tagPose.X().value());
    tagTable->GetEntry("tagDebug/tagPose/y").SetDouble(tagPose.Y().value());


//    visionTable->GetEntry("tagDebug/camPose/x").SetDouble(cameraPose.X().value());
//    visionTable->GetEntry("tagDebug/camPose/y").SetDouble(cameraPose.Y().value());
//    visionTable->GetEntry("tagDebug/camPose/theta").SetDouble(cameraPose.Rotation().Degrees().value());
}

frc::Twist2d VisionManager::getVelocityToTarget() const {
    return twistDelta;
}

void VisionManager::setLeds(bool state, int cameraIndex) {
    source->setLeds(state, cameraIndex);
}

//https://github.com/frc3512/Robot-2020/blob/main/src/main/cpp/controllers/TurretController.cpp#L209-L270
units::radian_t VisionManager::calculateHeadingAdjustment(const frc::Translation2d &globalTurretTranslation,
                                                          const frc::Velocity2d &drivetrainVelocity,
                                                          units::radians_per_second_t flywheelVel) const {
    static constexpr auto flywheelRad = 2_in;
    if(flywheelVel == 0_rad_per_s) return 0_rad;

    auto ballSpeed = flywheelVel * flywheelRad / 2.0 / 1_rad;
    frc::Translation2d targetPos(config.fieldToTarget.X(), config.fieldToTarget.Y());
    auto targetVelocity = -drivetrainVelocity;

    return units::math::asin(
            Cross(targetPos - globalTurretTranslation, targetVelocity) /
            ((targetPos - globalTurretTranslation).Norm() * ballSpeed)
    );
}

//https://github.com/frc3512/Robot-2020/blob/main/src/main/cpp/controllers/TurretController.cpp#L272-L302
units::radians_per_second_t VisionManager::angularVelocityToTarget() const {

    const auto v = getVelocityToTarget();
    frc::Translation2d vVec = {
            v.dx,
            v.dy
    };

    const auto r = config.fieldToTarget.Translation();

    return units::radians_per_second_t{
            VisionUtilities::Dot(frc::Translation2d{-r.Y(), r.X()} / VisionUtilities::Dot(r, r).value(), vVec)
                    .value()};
}

std::vector<std::shared_ptr<ChecklistItem>> VisionManager::createTests() {
    return {
            std::make_shared<SSHConnectionTest>("10.46.35.6")
    };
}
