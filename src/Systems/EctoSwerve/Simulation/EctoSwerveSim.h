//
// Created by abiel on 2/4/22.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOSWERVESIM_H
#define BOTBUSTERS_REBIRTH_ECTOSWERVESIM_H

#include <Core/EctoModule/System.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <Simulation/Swerve/SwerveModuleSim.h>

#include "Simulation/Swerve/QuadSwerveSim.h"
#include "Core/EctoModule/Manager.h"
#include "Core/MotorHandler/MotorManager.h"
#include <frc/smartdashboard/Field2d.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

struct EctoSwerveSimModule {
    std::shared_ptr<EctoSimulatedMotor> steerMotor, wheelMotor;
};

class EctoSwerveSim : public Manager {
public:
    EctoSwerveSim(const std::shared_ptr<EctoSwerve> &swerve);

    void resetPose(const frc::Pose2d &pose);

    void init() override;

    void update() override;
private:
    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    MotorManager &motorHandler = MotorManager::getInstance();

    std::unique_ptr<QuadSwerveSim> sim;
    std::shared_ptr<EctoSwerve> swerve;
    std::array<EctoSwerveSimModule, 4> moduleData;
    std::unique_ptr<frc::Field2d> field2d;
    const std::vector<std::string> motorPrefixes = {"front_left", "front_right",
                                              "back_left", "back_right"};
};


#endif //BOTBUSTERS_REBIRTH_ECTOSWERVESIM_H
