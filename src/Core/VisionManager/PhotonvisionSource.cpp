//
// Created by abiel on 2/25/22.
//

#include "PhotonvisionSource.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

PhotonvisionSource::PhotonvisionSource(const std::shared_ptr<EctoSwerve> &swerve) {
    this->swerve = swerve;
}

void PhotonvisionSource::setLeds(bool state, int cameraIndex) {
    auto camera = cameras[cameraIndex];
    camera.SetLEDMode(state ? photonlib::LEDMode::kOn : photonlib::LEDMode::kOff);

}

CameraInfo PhotonvisionSource::getCameraInfo() const {
    CameraInfo out;
    out.resolution = {640, 480}; //Photonvision does not publish camera info to nt
    out.horizontalFov = 59.6_deg;
    out.verticalFov = 49.7_deg;
    return out;
}

VisionResult PhotonvisionSource::getLatestResult() const {
//    VisionResult out;
//    const auto res = camera.GetLatestResult();
//    out.hasTargets = res.HasTargets();
//    out.ledState = camera.GetLEDMode() == photonlib::LEDMode::kOn;
//    out.timestamp = frc::Timer::GetFPGATimestamp() + res.GetLatency();
//    out.latency = res.GetLatency();
//    for(const auto &target : res.GetTargets()){
//        VisionTarget visionTarget;
//        for(const auto &point : target.GetCorners())
//            visionTarget.contourPoints.emplace_back(point);
//
//        out.targets.push_back(std::move(visionTarget));
//    }
//
    return {};
}

TagResult PhotonvisionSource::getLatestTagResult(){
    TagResult out;
    int i = 0;
    for (auto &camera : cameras) {
        photonlib::PhotonPipelineResult res = camera.GetLatestResult();
        out.timestamp = frc::Timer::GetFPGATimestamp() + res.GetLatency();
        out.hasTargets = res.HasTargets();
        out.latency = res.GetLatency();
        out.ledState = camera.GetLEDMode() == photonlib::LEDMode::kOn;
        auto targets = res.GetTargets();

        for (auto &target: targets) {
            AprilTag tag;
            tag.cameraId = i;
            tag.id = target.GetFiducialId();
            tag.tagPose = target.GetBestCameraToTarget();
            tag.timestamp = frc::Timer::GetFPGATimestamp() + res.GetLatency();
            tag.robotYaw = units::radian_t(swerve->getYaw());
            target.GetBestCameraToTarget();
            out.tags.emplace_back(tag);
        }
        i += 1;
    }
    return out;
}
