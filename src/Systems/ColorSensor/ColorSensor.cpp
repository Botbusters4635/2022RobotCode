//
// Created by cc on 03/08/22.
//

#include "ColorSensor.h"

ColorSensor::ColorSensor(const ColorSensorConfig &config) : WPISubsystem("colorSensor") {
    this->config = config;
}

void ColorSensor::robotUpdate() {
    r = table->GetNumber("rgb/r", 0);
    g = table->GetNumber("rgb/g", 0);
    b = table->GetNumber("rgb/b", 0);

}

std::vector<double> ColorSensor::getRGB() {
    return {r, g, b};
}

std::vector<double> ColorSensor::getHSV() const {
    double red, green, blue;
    double h, s, v;

    red = this->r / config.rgbScale;
    green = this->g / config.rgbScale;
    blue = this->b / config.rgbScale;

    double mx = std::max(red, std::max(green, blue));
    double mn = std::min(red, std::min(green, blue));
    double df = mx - mn;
    if (mx == mn){
        h = 0;
    }else if (mx == red){
        h = 60 * (fmod(((green - blue) / df), 6));
    }else if (mx == green){
        h = 60 * (((blue - red) / df) + 2);
    }else if (mx == blue){
        h = 60 * (((red - green) / df) + 4);
    }
    if (mx == 0){
        s = 0;
    } else{
        s = df/mx;
    }
    v = mx;
    return {h, s, v};
}

std::vector<double> ColorSensor::getCMYK() {
    if (r == 0 && g == 0 && b == 0){
        return {0, 0, 0, config.CMYKScale};
    }
    double c, m, y, k;
    c = 1 - r / config.rgbScale;
    m = 1 - g / config.rgbScale;
    y = 1 - b / config.rgbScale;

    double mn = std::min(c, std::min(m, y));

    c = (c - mn) / (1 - mn);
    m = (m - mn) / (1 - mn);
    y = (y - mn) / (1 - mn);
    k = mn;


    return {c * config.CMYKScale,
            m * config.CMYKScale,
            y * config.CMYKScale,
            k * config.CMYKScale};
}

