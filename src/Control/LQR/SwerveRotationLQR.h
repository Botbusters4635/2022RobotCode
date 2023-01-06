//
// Created by abiel on 4/12/22.
//

#ifndef BOTBUSTERS_REBIRTH_SWERVEROTATIONLQR_H
#define BOTBUSTERS_REBIRTH_SWERVEROTATIONLQR_H

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

class EctoSwerve;

class SwerveRotationLQR {
public:
    SwerveRotationLQR(const std::shared_ptr<EctoSwerve> &swerve, const frc::LinearSystem<2,1,1> &plant);

    void reset();

    units::volt_t calculate(units::radian_t positionSetpoint, units::radians_per_second_t velocitySetpoint);

    bool atSetpoint() const;
    units::radian_t estimatedPosition() const;
    units::radians_per_second_t estimatedVelocity() const;
private:
    void updateTelemetry();

    frc::LinearSystem<2,1,1> thetaPlant;
    frc::KalmanFilter<2,1,1> thetaObserver;
    frc::LinearQuadraticRegulator<2,1> thetaController;
    frc::LinearSystemLoop<2,1,1> thetaLoop;
    frc::LinearPlantInversionFeedforward<2,1> thetaFeedforward;

    std::shared_ptr<EctoSwerve> swerve;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    Eigen::Vector<double,1> u;
    units::radian_t lastYaw;
    Eigen::Matrix<double,2,1> error = {999999.0, 9999999.0};
};


#endif //BOTBUSTERS_REBIRTH_SWERVEROTATIONLQR_H
