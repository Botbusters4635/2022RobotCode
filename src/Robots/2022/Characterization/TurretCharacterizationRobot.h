//
// Created by cc on 08/06/22.
//

#ifndef BOTBUSTERS_REBIRTH_TURRETCHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_TURRETCHARACTERIZATIONROBOT_H

#include "Core/EctoCharacterizationRobot.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <sysid/logging/SysIdGeneralMechanismLogger.h>

class TurretCharacterizationRobot : public EctoCharacterizationRobot{
public:
    TurretCharacterizationRobot();

    void robotInit() override;
    void robotUpdate() override;
    void teleopUpdate() override;
    void autoInit() override;
    void autoUpdate() override;
    void disabledInit() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "turretMotor", 12}
        };
    };

    std::shared_ptr<EctoMotor> turretMotor;
    sysid::SysIdGeneralMechanismLogger logger;

};


#endif //BOTBUSTERS_REBIRTH_TURRETCHARACTERIZATIONROBOT_H
