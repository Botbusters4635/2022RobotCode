//
// Created by cc on 09/06/22.
//

#ifndef BOTBUSTERS_REBIRTH_TURRETTESTING_H
#define BOTBUSTERS_REBIRTH_TURRETTESTING_H

#include "Core/EctoRobot.h"
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <Systems/PIDTurret/PIDTurret.h>
#include <Commands/Turret/HomeTurret/HomeTurret.h>
#include <Commands/Turret/SetTurret/SetTurret.h>
#include <frc/ADIS16470_IMU.h>

class TurretTesting : public EctoRobot{
public:
    TurretTesting();

    void disabledInit() override;

    void disabledUpdate() override;

    void robotInit() override;

    void robotUpdate() override;

    void teleopInit() override;

    void teleopUpdate() override;

    void autoInit() override;

    void autoUpdate() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "turretMotor", 12}
        };
    };

    InputManager &input = InputManager::getInstance();


    std::shared_ptr<EctoMotor> turretMotor;
    std::shared_ptr<PIDTurret> turret;

    JoystickAxisExpo turretX{0.2, 0.2}, turretY{0.2, 0.2};
    EctoButton homeTurret;


    EctoButton up, down, left, right;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table, visionTable;
    nt::NetworkTableEntry visionError, hasTarget;

    frc::PIDController visionPID{1.2, 0, 0.000045};

//    std::unique_ptr<frc::ADIS16470_IMU> adis{};

//    double yaw;

};


#endif //BOTBUSTERS_REBIRTH_TURRETTESTING_H
