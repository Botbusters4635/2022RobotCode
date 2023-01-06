//
// Created by abiel on 2/25/22.
//

#ifndef BOTBUSTERS_REBIRTH_PHOTONVISIONSOURCE_H
#define BOTBUSTERS_REBIRTH_PHOTONVISIONSOURCE_H

#include "VisionSource.h"
#include <PhotonLib/PhotonCamera.h>
#include <PhotonLib/Packet.h>
#include <PhotonLib/PhotonUtils.h>
#include <PhotonLib/PhotonTrackedTarget.h>
#include <Systems/EctoSwerve/EctoSwerve.h>

class PhotonvisionSource : public VisionSource {
public:
    PhotonvisionSource(const std::shared_ptr<EctoSwerve> &swerve);

    void setLeds(bool state, int cameraIndex) override;

    CameraInfo getCameraInfo() const override;

    VisionResult getLatestResult() const override;

    TagResult getLatestTagResult() override;


private:

    std::shared_ptr<EctoSwerve> swerve;

    photonlib::PhotonCamera leftCamera{"leftOne"};//0
    photonlib::PhotonCamera rightCamera{"rightOne"};//1


    std::vector<photonlib::PhotonCamera> cameras{leftCamera, rightCamera};

    photonlib::PhotonPipelineResult prevLeftCamera;
    photonlib::PhotonPipelineResult prevRightCamera;
};


#endif //BOTBUSTERS_REBIRTH_PHOTONVISIONSOURCE_H
