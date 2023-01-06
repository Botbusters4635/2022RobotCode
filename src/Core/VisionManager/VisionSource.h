//
// Created by abiel on 2/25/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONSOURCE_H
#define BOTBUSTERS_REBIRTH_VISIONSOURCE_H

#include <vector>
#include <units/time.h>
#include <units/angle.h>
#include <frc/geometry/Transform3d.h>

struct VisionTarget {
    std::vector<std::pair<double,double>> contourPoints;
};

struct VisionResult {
    std::vector<VisionTarget> targets;
    bool hasTargets{false};
    bool ledState{false};
    units::second_t timestamp;
    units::second_t latency;
};

struct AprilTag{
    frc::Transform3d tagPose;
    units::second_t timestamp;
    units::radian_t robotYaw;
    int cameraId{};
    int id{-1};
};

struct TagResult{
    std::vector<AprilTag> tags;
    units::second_t timestamp;
    units::second_t latency;
    bool hasTargets{false};
    bool ledState{false};
};

struct CameraInfo {
    std::pair<double,double> resolution;
    units::degree_t verticalFov, horizontalFov;

};



class VisionSource {
public:
    virtual void setLeds(bool state, int cameraIndex) = 0;

    [[nodiscard]] virtual CameraInfo getCameraInfo() const = 0;

    [[nodiscard]] virtual VisionResult getLatestResult() const = 0;

    virtual TagResult getLatestTagResult() = 0;
};

#endif //BOTBUSTERS_REBIRTH_VISIONSOURCE_H
