//
// Created by abiel on 2/25/22.
//

#ifndef BOTBUSTERS_REBIRTH_LIMELIGHTSOURCE_H
#define BOTBUSTERS_REBIRTH_LIMELIGHTSOURCE_H

#include "VisionSource.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class LimelightSource : public VisionSource {
public:
    LimelightSource();

    void setLeds(bool state, int cameraIndex) override;

    [[nodiscard]] CameraInfo getCameraInfo() const override;

    [[nodiscard]] VisionResult getLatestResult() const override;

private:
    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;
};


#endif //BOTBUSTERS_REBIRTH_LIMELIGHTSOURCE_H
