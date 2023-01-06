#include "gtest/gtest.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"

TEST(JoystickAxisExpo, deadzone){
    JoystickAxisExpo joy(0.1, 0.5);
    for(double i = -1; i <= 1.0; i += 0.05){
        if(std::abs(i) < 0.5){
            ASSERT_EQ(0, joy.calculateOutput(i));
        }
    }
}

TEST(JoystickAxisExpo, limits){
    JoystickAxisExpo joy(0.1, 0.5);
    ASSERT_EQ(1, joy.calculateOutput(1));
    ASSERT_EQ(-1, joy.calculateOutput(-1));
}