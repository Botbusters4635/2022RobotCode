//
// Created by abiel on 1/16/22.
//

#ifndef BOTBUSTERS_REBIRTH_ELEVATORTESTING_H
#define BOTBUSTERS_REBIRTH_ELEVATORTESTING_H

#include "Core/EctoRobot.h"
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <Core/EctoInput/Buttons/ToggleButton.h>

class ElevatorTesting : public EctoRobot {
public:
    ElevatorTesting();


    void teleopInit();

    void teleopUpdate();

    void updateTelemetry();

    double radToHeight(double rads);

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "elevator",               12},
                {EctoMotorType::SparkMax, "elevatorFollower",       11},
                {EctoMotorType::SparkMax, "elevatorFollowerSecond", 10}
        };
    }
//    std::list<PistonInfo> getPistonConfig(){
//        return {
//                {"gearBox", 20, frc::PneumaticsModuleType::REVPH, 0, 1}
//        };
//}
//    PCMManager &pcm = PCMManager::getInstance();


    std::shared_ptr<EctoMotor> elevator, elevator1, elevator2;

    double expo = 0.2;
    double deadzone = 0.2;
    JoystickAxisExpo frontAxis{expo, deadzone}, backAxis{expo, deadzone};
    ToggleButton gearBoxButton;
};


#endif //BOTBUSTERS_REBIRTH_ELEVATORTESTING_H
