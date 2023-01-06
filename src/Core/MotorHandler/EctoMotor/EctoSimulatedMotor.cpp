//
// Created by abiel on 2/8/22.
//

#include "EctoSimulatedMotor.h"

void EctoSimulatedMotor::factoryReset() {

}

void EctoSimulatedMotor::setLimitSwitchPolarity(bool switchPolarity) {

}

void EctoSimulatedMotor::setPosition(double pos) {
    position = pos;
}

void EctoSimulatedMotor::setVelocity(double vel) {
    velocity = vel;
}

double EctoSimulatedMotor::getPosition() const {
    return position;
}

double EctoSimulatedMotor::getVelocity() const {
    return velocity;
}

std::string EctoSimulatedMotor::getFirmwareVersion() const {
    return std::string("SIMULATED");
}

void EctoSimulatedMotor::invertMotor(bool state) {
    ;
}

bool EctoSimulatedMotor::isMotorInverted() const {
    return false;
}

void EctoSimulatedMotor::invertSensor(bool state) {
    ;
}

bool EctoSimulatedMotor::isSensorInverted() const {
    return false;
}

void EctoSimulatedMotor::setPIDConfig(const PIDConfig &pidConfig, int profileSlot) {

}

void EctoSimulatedMotor::enableBrakingOnIdle(bool state) {

}

void EctoSimulatedMotor::enableCurrentLimit(bool state) {

}

void EctoSimulatedMotor::setMotorCurrentLimit(double amps) {

}

void EctoSimulatedMotor::setClosedLoopOutputRange(double minimum, double maximum) {

}

void EctoSimulatedMotor::setClosedLoopRampRate(double rampRate) {

}

void EctoSimulatedMotor::setOpenLoopRampRate(double rampRate) {

}

void EctoSimulatedMotor::setSensorPosition(double position) {

}

double EctoSimulatedMotor::getTemperature() const {
    return 0;
}

double EctoSimulatedMotor::getCurrent() const {
    return currentDraw;
}

double EctoSimulatedMotor::getVoltage() const {
    return 12;
}

double EctoSimulatedMotor::getOutputPercent() const {
    return voltageOutput / 12.0;
}

void EctoSimulatedMotor::setEncoderCodesPerRev(int codes) {
    ;
}

int EctoSimulatedMotor::getEncoderCodesPerRev() const {
    return 0;
}

void EctoSimulatedMotor::setArbitraryFeedForward(double feedForward) {
    ;
}

void EctoSimulatedMotor::disable() {
    ;
}

bool EctoSimulatedMotor::isDisabled() const {
    return false;
}

void EctoSimulatedMotor::enableLimitSwitches(bool state) {

}

bool EctoSimulatedMotor::getForwardLimitSwitch() const {
    return false;
}

bool EctoSimulatedMotor::getReverseLimitSwitch() const {
    return std::abs(getPosition()) < 0.25;
}

void EctoSimulatedMotor::setForwardSoftLimit(double radians) {

}

void EctoSimulatedMotor::enableForwardSoftLimit(bool state) {

}

void EctoSimulatedMotor::setReverseSoftLimit(double radians) {

}

void EctoSimulatedMotor::enableReverseSoftLimit(bool state) {

}

void EctoSimulatedMotor::configureMotionMagicVelocity(double velocity) {

}

void EctoSimulatedMotor::configureMotionMagicAcceleration(double acceleration) {

}

void EctoSimulatedMotor::configureMotionMagicSCurve(double sCurve) {

}

void EctoSimulatedMotor::setAnalogPositionConversionFactor(double conversionFactor) {

}

void EctoSimulatedMotor::setAnalogVelocityConversionFactor(double conversionFactor) {

}

void EctoSimulatedMotor::setAnalogSensorOffset(double analogVoltageOffset) {

}

void EctoSimulatedMotor::followMotor(const EctoMotor &masterMotor, bool isInverted) {

}

void EctoSimulatedMotor::enableVoltageCompensation(double nominalVoltage) {

}

void EctoSimulatedMotor::prioritizeUpdateRate() {

}

void EctoSimulatedMotor::deprioritizeUpdateRate() {
    ;
}

void EctoSimulatedMotor::setCANTimeout(int milliseconds) {
   ;
}

void EctoSimulatedMotor::outputSet(double set) {
    ;
}

bool EctoSimulatedMotor::runFaultTest() {
    return true;
}

double EctoSimulatedMotor::getRawAnalogPosition() const {
    return 0;
}

double EctoSimulatedMotor::getPotPosition() const {
    return 0;
}

double EctoSimulatedMotor::getPotVelocity() const {
    return 0;
}

double EctoSimulatedMotor::getQuadPosition() const {
    return 0;
}

double EctoSimulatedMotor::getQuadVelocity() const {
    return 0;
}

units::volt_t EctoSimulatedMotor::getOutputVoltage() const {
    return units::volt_t(voltageOutput);
}

void EctoSimulatedMotor::setVoltageOutput(double voltage) {
    voltageOutput = voltage;
}

void EctoSimulatedMotor::setMotorOutputByCurrent(double amps) {

}

void EctoSimulatedMotor::setOutputPercent(double value) {
    voltageOutput = value * 12.0;
}

void EctoSimulatedMotor::setPositionSetpoint(double position) {

}

void EctoSimulatedMotor::setVelocitySetpoint(double velocity) {

}

void EctoSimulatedMotor::setMotionMagicOutput(double value) {

}

void EctoSimulatedMotor::setPotAsClosedLoopSource() {

}

void EctoSimulatedMotor::setQuadAsClosedLoopSource() {

}
