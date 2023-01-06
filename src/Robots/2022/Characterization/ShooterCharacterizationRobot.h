//
// Created by cc on 30/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_SHOOTERCHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_SHOOTERCHARACTERIZATIONROBOT_H

#include "Core/EctoRobot.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include "Systems/EctoSwerve/EctoSwerve.h"

#include "Core/EctoCharacterizationRobot.h"
#include <sysid/logging/SysIdGeneralMechanismLogger.h>

class ShooterCharacterizationRobot : public EctoCharacterizationRobot {
public:
    ShooterCharacterizationRobot();

    void robotInit() override;

    void robotUpdate() override;

    void teleopUpdate() override;

    void autoInit() override;

    void autoUpdate() override;

    void disabledInit() override;


private:
    double gearRatio = 1.5;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "shooterMotor",  16},
                {EctoMotorType::SparkMax, "invertedShooterMotor", 15},

        };
    };

    std::shared_ptr<EctoMotor> masterMotor, slaveMotor;

    sysid::SysIdGeneralMechanismLogger logger;
};


#endif //BOTBUSTERS_REBIRTH_SHOOTERCHARACTERIZATIONROBOT_H
