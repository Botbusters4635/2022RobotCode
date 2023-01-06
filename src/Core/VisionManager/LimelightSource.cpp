//
// Created by abiel on 2/25/22.
//

#include "LimelightSource.h"
#include <frc/Timer.h>

LimelightSource::LimelightSource() {
    table = ntInstance.GetTable("limelight");
}

void LimelightSource::setLeds(bool state, int cameraIndex) {
    table->GetEntry("ledMode").SetDouble(state ? 3 : 1);

}

CameraInfo LimelightSource::getCameraInfo() const {
    CameraInfo out;
    out.resolution = {960, 720}; //Limelight does not publish camera info to nt
    out.horizontalFov = 59.6_deg;
    out.verticalFov = 49.7_deg;
    return out;
}

VisionResult LimelightSource::getLatestResult() const {
    VisionResult out;
    out.hasTargets = table->GetEntry("tv").GetDouble(0) == 1;
    out.ledState = table->GetEntry("ledMode").GetDouble(0) == 3;
    //The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
    out.latency = units::millisecond_t(table->GetEntry("tl").GetDouble(0)) + 11_ms;
    out.timestamp = frc::Timer::GetFPGATimestamp() - out.latency;
    std::vector<double> rawCorners = table->GetEntry("tcornxy").GetDoubleArray({});
    if(rawCorners.size() % 8 != 0) return out;

    for(size_t i = 0; i < rawCorners.size(); i += 8){
        VisionTarget target;
        target.contourPoints.emplace_back(std::make_pair(rawCorners.at(i), rawCorners.at(i+1)));
        target.contourPoints.emplace_back(std::make_pair(rawCorners[i+2], rawCorners[i+3]));
        target.contourPoints.emplace_back(std::make_pair(rawCorners[i+4], rawCorners[i+5]));
        target.contourPoints.emplace_back(std::make_pair(rawCorners[i+6], rawCorners[i+7]));
        out.targets.push_back(std::move(target));
    }

    return out;
}
