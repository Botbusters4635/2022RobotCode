//
// Created by abiel on 3/20/22.
//

#ifndef BOTBUSTERS_REBIRTH_SHOOTER_H
#define BOTBUSTERS_REBIRTH_SHOOTER_H

#include <Core/EctoModule/WPISubsystem.h>
#include <frc/Servo.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class Shooter : public WPISubsystem {
public:
    Shooter(const std::string &name, std::initializer_list<int> servo_ids);

    virtual double getVelocity() const = 0;

    virtual void setSetpoint(double velocity) = 0;

    void setHood(double angle);

    units::radian_t getEstimatedHoodAngle();

    virtual bool flywheelAtSetpoint() = 0;

    bool hoodAtSetpoint();

private:
    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    std::vector<std::shared_ptr<frc::Servo>> hoodServos;
    units::radian_t hoodSetpoint{25_deg};
    frc::SlewRateLimiter<units::radian> hoodAngleEstimator{1.0_rad_per_s};
};

#endif //BOTBUSTERS_REBIRTH_SHOOTER_H
