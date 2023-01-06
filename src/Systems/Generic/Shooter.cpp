//
// Created by abiel on 3/20/22.
//

#include <units/math.h>
#include "Shooter.h"

Shooter::Shooter(const std::string &name, std::initializer_list<int> servo_ids) : WPISubsystem(name) {
    for(const auto id : servo_ids){
        hoodServos.emplace_back(std::make_shared<frc::Servo>(id));
        hoodServos.back()->SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }

    table = ntInstance.GetTable("Shooter");
    setHood(0);
    hoodAngleEstimator.Reset(0_rad);
}

void Shooter::setHood(double angle) {
    if(angle < 0.0){
        return;
    }
    table->GetEntry("HoodSetPoint").SetDouble(angle);
    for(const auto& servo : hoodServos){
        servo->Set(std::clamp(angle, 0.0, 1.0));
    }

    hoodSetpoint = units::radian_t(angle);
}

units::radian_t Shooter::getEstimatedHoodAngle() {
    return hoodAngleEstimator.Calculate(hoodSetpoint);
}

bool Shooter::hoodAtSetpoint() {
    return units::math::abs(getEstimatedHoodAngle() - hoodSetpoint) < 10_rad;
}