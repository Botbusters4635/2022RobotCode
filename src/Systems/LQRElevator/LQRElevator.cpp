////
//// Created by cc on 23/05/22.
////
//
//#include "LQRElevator.h"
//
//LQRElevator::LQRElevator(const LQRElevatorConfig &config, const std::shared_ptr<GearBox> &gearBox) : WPISubsystem("LQRElevator"),
//                                                            slewRateLimiter(config.maxAccel),
//                                                            elevatorPlant(frc::LinearSystemId::IdentifyPositionSystem<units::meter>(config.kV / 1_mps,
//                                                                                                                                  config.kA / 1_mps_sq)),
//                                                            observer(elevatorPlant,
//                                                                     {0.1, 1.0},
//                                                                     {1.0},
//                                                                     20_ms),
//                                                            controller(elevatorPlant,
//                                                                       {0.03, 0.3},
//                                                                       {config.maxControllVoltage.value()},
//                                                                       20_ms),
//                                                            loop(elevatorPlant, controller, observer, config.maxControllVoltage, 20_ms),
//                                                            constraints(config.maxVel, config.maxAccel){
//    table = ntInstance.GetTable("LQRElevator");
//    this->config = config;
//    this->motors = config.motors;
//    this->gearBox = gearBox;
//
//    motors[0]->setControlMode(MotorControlMode::Voltage);
//    motors[0]->setMotorCurrentLimit(config.currentLimit.value());
//    motors[0]->enableCurrentLimit(true);
//    motors[0]->enableBrakingOnIdle(true);
//    motors[0]->setClosedLoopRampRate(config.rampRate);
//    motors[0]->setFeedbackMode(MotorFeedbackMode::QuadEncoder);
//    motors[0]->setForwardSoftLimit(heightToRad(config.forwardSoftLimit).value());
//    motors[0]->setReverseSoftLimit(heightToRad(config.reverseSoftLimit).value());
//    motors[0]->enableForwardSoftLimit(config.enableForwardSoftLimit);
//    motors[0]->enableReverseSoftLimit(config.enableReverseSoftLimit);
//
//
//    if (config.isInverted.size() != motors.size())
//        throw std::runtime_error("Motor inversion not configured!");
//
//    for (size_t i = 0; i < motors.size(); i++) {
//        const auto motor = motors[i];
//        if (i == 0) {
//            motor->invertMotor(config.isInverted[i]);
//        } else {
//            motor->followMotor(*motors[0], config.isInverted[i]);
//            motor->deprioritizeUpdateRate();
//        }
//    }
//
//    lastProfiledReference = {getHeight(), getVel()};
//
//    controller.LatencyCompensate(elevatorPlant, 20_ms, 25_ms);
//
//    loop = frc::LinearSystemLoop<2,1,1>(elevatorPlant, controller, observer, config.maxControllVoltage, 20_ms);
//
//    loop.Reset(Eigen::Vector<double, 2>{0.0, 0.0});
//
//    lastProfiledReference = {getHeight(), getVel()};
//
//}
//
//void LQRElevator::set(units::meter_t height) {
//    targetSetpoint = height;
//}
//
//units::meter_t LQRElevator::radToHeight(units::radian_t rads) const{
//    rads /= config.gearRatio;
//    double motorRotations = rads.value() / (2.0 * M_PI);
//    return units::meter_t(motorRotations * (config.pulleyDiameter.value() * M_PI));
//}
//
//units::radian_t LQRElevator::heightToRad(units::meter_t height) const{
//    double rots = (height.value() / (config.pulleyDiameter.value() * M_PI));
//    rots = config.gearRatio * rots;
//    return units::radian_t(rots * (2.0 * M_PI));
//}
//
//units::meter_t LQRElevator::getHeight() {
//    return radToHeight(units::radian_t(motors[0]->getPosition()));
//}
//
//units::meters_per_second_t LQRElevator::getVel() {
//    return radToHeight(units::radian_t(motors[0]->getVelocity())) / 1_s;
//}
//
//bool LQRElevator::getForwardLimitSwitch() {
//    return motors[0]->getForwardLimitSwitch();
//}
//
//bool LQRElevator::getReverseLimitSwitch() {
//    motors[0]->getReverseLimitSwitch();
//}
//
//void LQRElevator::resetToZero() {
//    motors[0]->setSensorPosition(0);
//}
//
//void LQRElevator::setSensorAngle(units::meter_t height) {
//    motors[0]->setSensorPosition(heightToRad(height).value());
//}
//
//void LQRElevator::useSoftLimits(bool set) {
//    motors[0]->enableForwardSoftLimit(set);
//    motors[0]->enableReverseSoftLimit(set);
//    table->GetEntry("usingSoftLimits").SetBoolean(set);
//}
//
//void LQRElevator::updateTelemtry() {
//    table->GetEntry("position").SetDouble(getHeight().value());
//    table->GetEntry("vel").SetDouble(getVel().value());
//    table->GetEntry("setPoint").SetDouble(targetSetpoint.value());
//}
//
//void LQRElevator::setVoltage(units::volt_t setVoltage){manualVoltage = setVoltage.value();}
//
//void LQRElevator::robotInit() {;}
//
//void LQRElevator::robotUpdate() {
//
//
//}