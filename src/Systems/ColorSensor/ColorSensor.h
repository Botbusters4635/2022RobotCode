//
// Created by cc on 03/08/22.
//

#ifndef BOTBUSTERS_REBIRTH_COLORSENSOR_H
#define BOTBUSTERS_REBIRTH_COLORSENSOR_H

#include "Core/EctoModule/WPISubsystem.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

//enum class ColorProfile {
//    rgb,
//    cmyk,
//    hsv
//};

struct ColorSensorConfig{
    double rgbScale = 255;
    double CMYKScale = 100;
};

class ColorSensor : public WPISubsystem{
public:
    ColorSensor(const ColorSensorConfig &config);

    void robotUpdate();

    std::vector<double> getRGB();

    std::vector<double> getCMYK();

    std::vector<double> getHSV() const;

private:

    double r{}, g{}, b{};

    ColorSensorConfig config;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("ColorSensor");
};


#endif //BOTBUSTERS_REBIRTH_COLORSENSOR_H
