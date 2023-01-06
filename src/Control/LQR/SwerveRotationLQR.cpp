//
// Created by abiel on 4/12/22.
//

#include "SwerveRotationLQR.h"
#include "Systems/EctoSwerve/EctoSwerve.h"

//TODO make this configurable
SwerveRotationLQR::SwerveRotationLQR(const std::shared_ptr<EctoSwerve> &swerve,
                                     const frc::LinearSystem<2, 1, 1> &plant) :
        thetaPlant(plant),
        thetaObserver(thetaPlant,
                      {1.2155, 0.652},
                      {0.001},
                      20_ms),
        thetaController(thetaPlant,
                        {0.355, 1.73575},
                        {5.675},
                        20_ms),
        thetaLoop(thetaPlant, thetaController, thetaObserver, 10_V, 20_ms),
        thetaFeedforward(thetaPlant, 20_ms) {
    this->swerve = swerve;

    //thetaController.LatencyCompensate(thetaPlant, 20_ms, 1_ms);
//    thetaLoop = frc::LinearSystemLoop<2,1,1>(thetaPlant, thetaController,
//                                             thetaObserver, 10_V, 20_ms);

    table = ntInstance.GetTable("SwerveRotationLQR");
}

void SwerveRotationLQR::reset() {
    std::cout << "reset" << std::endl;
    thetaController.Reset();
    thetaObserver.Reset();
    thetaFeedforward.Reset();
    thetaLoop.Reset(Eigen::Vector<double,2>{swerve->getYaw(), swerve->getYawRate()});
}

units::volt_t
SwerveRotationLQR::calculate(units::radian_t positionSetpoint, units::radians_per_second_t velocitySetpoint) {
    auto pose = swerve->getPose();
    auto yaw = swerve->getYaw();

    thetaLoop.SetNextR(Eigen::Vector<double,2>{
        positionSetpoint.value(),
        velocitySetpoint.value()
    });

    error = thetaLoop.Controller().R() - Eigen::Vector<double,2>(yaw,0.0);
    error(0,0) = frc::AngleModulus(error(0,0) * 1_rad).value();
    u = thetaLoop.Controller().K() * error;
    u += thetaFeedforward.Calculate(thetaLoop.NextR());
    u = thetaLoop.ClampInput(u);
    thetaController.Calculate(thetaLoop.Xhat(), thetaLoop.NextR());
    thetaObserver.Correct(u, Eigen::Vector<double,1>{yaw});
    thetaObserver.Predict(u, 20_ms);

    lastYaw = yaw * 1_rad;
    updateTelemetry();

    return u(0,0) * 1_V;
}

units::radian_t SwerveRotationLQR::estimatedPosition() const {
    return thetaLoop.Xhat(0) * 1_rad;
}

units::radians_per_second_t SwerveRotationLQR::estimatedVelocity() const {
    return thetaLoop.Xhat(1) * 1_rad_per_s;
}

bool SwerveRotationLQR::atSetpoint() const {
    return std::abs(error(0,0)) < 0.1 and std::abs(error(1,0)) < 0.1;
}


void SwerveRotationLQR::updateTelemetry() {
    table->GetEntry("Setpoint/Position").SetDouble(thetaLoop.NextR(0));
    table->GetEntry("Setpoint/Velocity").SetDouble(thetaLoop.NextR(1));
    table->GetEntry("Measurment/Position").SetDouble(lastYaw.value());
    table->GetEntry("Measurment/Velocity").SetDouble(swerve->getYawRate());
    table->GetEntry("StateEstimate/Position").SetDouble(thetaLoop.Xhat(0));
    table->GetEntry("StateEstimate/Velocity").SetDouble(thetaLoop.Xhat(1));
    table->GetEntry("Error/Position").SetDouble(thetaLoop.Controller().R(0) - lastYaw.value());
    table->GetEntry("ControlEffort").SetDouble(u(0,0));
}