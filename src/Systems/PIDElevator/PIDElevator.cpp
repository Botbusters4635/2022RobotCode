//
// Created by abiel on 1/16/22.
//

#include "PIDElevator.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

PIDElevator::PIDElevator(const PIDElevatorConfig &config)
    : WPISubsystem("PIDElevator") {
  this->config = config;
  this->motors = config.motors;

  table = ntInstance.GetTable(fmt::format("PIDElevator/{}", "PIDElevator"));

  frc::ProfiledPIDController<units::meters>::Constraints constraints(config.maxVelocity,
                                                                     config.maxAcceleration);

  pidController = std::make_unique<frc::ProfiledPIDController<units::meters>>(
      config.pidConfig.p, config.pidConfig.i, config.pidConfig.d, constraints);

  pidController->SetTolerance(config.pidDistTol,
                              config.pidVelTol);
  pidController->Reset(getHeight());

  if (motors.empty()) {
    log->error("No motors given to PIDElevator!!!");
    throw std::runtime_error("Motor vector is empty!");
  }

  motors[0]->setControlMode(MotorControlMode::Voltage);
  motors[0]->setOpenLoopRampRate(config.rampRate);
  motors[0]->setMotorCurrentLimit(config.currentLimit.value());
  motors[0]->enableCurrentLimit(true);

  motors[0]->setReverseSoftLimit(heightToRad(config.reverseSoftLimit).value());
  motors[0]->setForwardSoftLimit(heightToRad(config.forwardSoftLimit).value());
  motors[0]->enableForwardSoftLimit(true);
  motors[0]->enableReverseSoftLimit(true);

  motors[0]->enableBrakingOnIdle(true);
  motors[0]->enableLimitSwitches(config.enableLimitSwitch);
  motors[0]->setFeedbackMode(MotorFeedbackMode::QuadEncoder);

  if (config.isInverted.size() != motors.size())
    throw std::runtime_error("Motor inversion not configured!");

  for (size_t i = 0; i < motors.size(); i++) {
    const auto motor = motors[i];
    if (i == 0) {
      motor->invertMotor(config.isInverted[i]);
    } else {
      motor->followMotor(*motors[0], config.isInverted[i]);
      motor->deprioritizeUpdateRate();
    }
  }

  pidController->Reset(getHeight());
  setUsedFF(PIDElevator::FeedForward::Free);

  masterMotorCurrent = table->GetEntry("masterMotorCurrent");
  slaveMotorCurrent = table->GetEntry("salveMotorCurrent");

}

void PIDElevator::robotInit() { ; }

void PIDElevator::setPIDConfig(const PIDConfig &pidConfig) {
  pidController->SetPID(pidConfig.p, pidConfig.i, pidConfig.d);
}

void PIDElevator::set(units::meter_t height) {
//    if(gearBox->engaged()){log->warn("tried to set height while gearBox engaged"); return;}
    pidController->SetGoal(height);
}

units::meter_t PIDElevator::radToHeight(units::radian_t rads) const{
  double motorRotations = rads.value() / (2.0 * M_PI);
  motorRotations /= config.gearReduction;
  return units::meter_t(motorRotations * (config.pulleyDiameter.value() * M_PI));
}

units::radian_t PIDElevator::heightToRad(units::meter_t height) const{
  double rots = (height.value() / (config.pulleyDiameter.value() * M_PI));
  rots = config.gearReduction * rots;
  return units::radian_t(rots * (2.0 * M_PI));
}

void PIDElevator::setVoltage(double setVoltage) { manualVoltage = setVoltage; }

bool PIDElevator::getLimitSwitchState() {
  return motors[0]->getReverseLimitSwitch();
}

void PIDElevator::setUsePID(bool usePID) {
  if (this->usePID != usePID && usePID) {
    pidController->Reset(getHeight());
  }
  this->usePID = usePID;
}

void PIDElevator::resetToZero() { motors[0]->setSensorPosition(0); }

bool PIDElevator::atGoal() { return pidController->AtGoal(); }

void PIDElevator::setUsedFF(FeedForward ff) {
  switch (ff) {
    case PIDElevator::FeedForward::Free:
      this->ff = &this->config.freeFF;
      break;
    case PIDElevator::FeedForward::Loaded:
      this->ff = &this->config.loadedFF;
      break;
    default:

      break;
  }
}

void PIDElevator::useSoftLimits(bool useSoftLimits) {
    motors[0]->enableForwardSoftLimit(useSoftLimits);
    motors[0]->enableReverseSoftLimit(useSoftLimits);
    table->GetEntry("usingSoftLimits").SetBoolean(useSoftLimits);

}

units::meter_t PIDElevator::getHeight() const {
  auto motorRads = units::radian_t(motors[0]->getPosition());
  return units::meter_t(radToHeight(motorRads));
}

void PIDElevator::resetController(units::meter_t height){
    pidController->Reset(height);
}

void PIDElevator::robotUpdate() {
//  if (gearBox->engaged()){return;}
    double out{};
  if (usePID) {
      out = pidController->Calculate(getHeight());
    out += ff->Calculate(pidController->GetSetpoint().velocity).value();
    motors[0]->set(out, MotorControlMode::Voltage);
  } else {
    motors[0]->set(manualVoltage, MotorControlMode::Voltage);
  }
  table->GetEntry("MotorSetpoint")
      .SetDouble(pidController->GetSetpoint().position.value());
  //table->GetEntry("VoltageApplied").SetDouble(motors[0]->getVoltage());
  frc::SmartDashboard::PutData(fmt::format("PIDElevator/{}", getName()),
                               pidController.get());

  masterMotorCurrent.SetDouble(motors[0]->getCurrent());
  slaveMotorCurrent.SetDouble(motors[1]->getCurrent());
  table->GetEntry("MotorPostition").SetDouble(getHeight().value());
    table->GetEntry("Goal")
            .SetDouble(pidController->GetGoal().position.value());

    table->GetEntry("VoltageApplied").SetDouble(out);
    frc::SmartDashboard::PutData("PIDClimber/PIDClimber",
                                 pidController.get());

    masterMotorCurrent.SetDouble(motors[0]->getCurrent());
    slaveMotorCurrent.SetDouble(motors[1]->getCurrent());

    table->GetEntry("ClimberPosition").SetDouble(getHeight().value());
    table->GetEntry("UsePID").SetBoolean(usePID);
}