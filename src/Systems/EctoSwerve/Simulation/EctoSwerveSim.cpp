//
// Created by abiel on 2/4/22.
//

#include "EctoSwerveSim.h"
#include "Math/EctoMath.h"
#include <frc/system/plant/DCMotor.h>
#include <frc/smartdashboard/SmartDashboard.h>

EctoSwerveSim::EctoSwerveSim(const std::shared_ptr<EctoSwerve> &swerve) : Manager("EctoSwerveSim") {
    std::array<SwerveModuleSim, 4> modules;

    for (int i = 0; i < 4; i++) {
        auto module = SwerveModuleSim(
                frc::DCMotor::NEO(),
                frc::DCMotor::NEO(),
                units::inch_t(4) / 2,
                40.0, //aziGearRatio
                8.33, //wheelGearRatio
                1, //aziEncGearRatio
                1, //WheelEncGearRatio
                1.1,
                0.8,
                units::newton_t(70 * 9.81 / 4),
                units::kilogram_square_meter_t(0.005)
        );

        modules[i] = module;
    }


    sim = std::make_unique<QuadSwerveSim>(
            units::meter_t(1),
            units::meter_t(1),
            units::kilogram_t(70),
            units::kilogram_square_meter_t(12),
            std::move(modules)
    );

    sim->modelReset(frc::Pose2d(units::meter_t(0), units::meter_t(0), frc::Rotation2d()));

    for (size_t i = 0; i < 4; i++) {
        const auto prefix = motorPrefixes[i];
        auto steerName = fmt::format("{}_steer", prefix);
        auto wheelName = fmt::format("{}_wheel", prefix);
        moduleData[i].steerMotor = motorHandler.getSimulatedMotor(steerName);
        moduleData[i].wheelMotor = motorHandler.getSimulatedMotor(wheelName);
    }

    table = ntInstance.GetTable("EctoSwerveSim");
    field2d = std::make_unique<frc::Field2d>();
    frc::SmartDashboard::PutData("SimSwervePose", field2d.get());

    this->swerve = swerve;
}

void EctoSwerveSim::init() {

}

void EctoSwerveSim::update() {
    /**
     * Update model modules
     */
    for(size_t i = 0; i < 4; i++){
        SwerveModuleSim &module(sim->getModule(i));
        EctoSwerveSimModule &motors(moduleData[i]);

        module.setInputVoltages(motors.wheelMotor->getOutputVoltage(), motors.steerMotor->getOutputVoltage());
        motors.wheelMotor->setPosition(module.getWheelEncoderPositionRev() * (2.0 * M_PI) * 8.33);
        motors.wheelMotor->setVelocity(module.getWheelMotorVelocity().value());
        motors.steerMotor->setPosition(EctoMath::wrapAngle(module.getAzimuthEncoderPositionRev() * (2.0 * M_PI)));
    }

    sim->update(units::millisecond_t(20));
    swerve->setYaw(sim->getCurrentPose().Rotation().Radians().value());

    for(int i = 0; i < 4; i++){
        auto &module = sim->getModule(i);
        auto prefix = motorPrefixes.at(i);
        auto subtable = table->GetSubTable(prefix);
        auto &motors = (moduleData.at(i));
        subtable->GetEntry("WheelVoltage").SetDouble(module.getWheelVoltage().value());
        subtable->GetEntry("AzmthVoltage").SetDouble(module.getAzmthVoltage().value());
        subtable->GetEntry("AzmthEncoderPosition").SetDouble(motors.steerMotor->getPosition());
        subtable->GetEntry("MotorEncoderPosition").SetDouble(motors.wheelMotor->getPosition());
    }

    table->GetEntry("X").SetDouble(sim->getCurrentPose().X().value());
    table->GetEntry("Y").SetDouble(sim->getCurrentPose().Y().value());
    table->GetEntry("Theta").SetDouble(sim->getCurrentPose().Rotation().Radians().value());
    field2d->SetRobotPose(sim->getCurrentPose());
    //field2d->SetRobotPose(sim->getCurrentPose());
}

void EctoSwerveSim::resetPose(const frc::Pose2d &pose) {
    sim->modelReset(pose);
}