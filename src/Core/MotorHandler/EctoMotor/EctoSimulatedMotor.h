//
// Created by abiel on 2/8/22.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOSIMULATEDMOTOR_H
#define BOTBUSTERS_REBIRTH_ECTOSIMULATEDMOTOR_H

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"

class EctoSimulatedMotor : public EctoMotor {
public:
    EctoSimulatedMotor(int id, const std::string &name) : EctoMotor(id, name, EctoMotorType::Simulated, false){
        ;
    }

    void factoryReset() override;

    units::volt_t getOutputVoltage() const;

    void setLimitSwitchPolarity(bool switchPolarity) override;

    void setPosition(double pos);

    double getPosition() const override;

    void setVelocity(double vel);

    double getVelocity() const override;

    std::string getFirmwareVersion() const override;

    void invertMotor(bool state) override;

    bool isMotorInverted() const override;

    void invertSensor(bool state) override;

    bool isSensorInverted() const override;

    void setPIDConfig(const PIDConfig &pidConfig, int profileSlot) override;

    void enableBrakingOnIdle(bool state) override;

    void enableCurrentLimit(bool state) override;

    void setMotorCurrentLimit(double amps) override;

    void setClosedLoopOutputRange(double minimum, double maximum) override;

    void setClosedLoopRampRate(double rampRate) override;

    void setOpenLoopRampRate(double rampRate) override;

    void setSensorPosition(double position) override;

    double getTemperature() const override;

    double getCurrent() const override;

    double getVoltage() const override;

    double getOutputPercent() const override;

    void setEncoderCodesPerRev(int codes) override;

    int getEncoderCodesPerRev() const override;

    void setArbitraryFeedForward(double feedForward) override;

    void disable() override;

    bool isDisabled() const override;

    void enableLimitSwitches(bool state) override;

    bool getForwardLimitSwitch() const override;

    bool getReverseLimitSwitch() const override;

    void setForwardSoftLimit(double radians) override;

    void enableForwardSoftLimit(bool state) override;

    void setReverseSoftLimit(double radians) override;

    void enableReverseSoftLimit(bool state) override;

    void configureMotionMagicVelocity(double velocity) override;

    void configureMotionMagicAcceleration(double acceleration) override;

    void configureMotionMagicSCurve(double sCurve) override;

    void setAnalogPositionConversionFactor(double conversionFactor) override;

    void setAnalogVelocityConversionFactor(double conversionFactor) override;

    void setAnalogSensorOffset(double analogVoltageOffset) override;

    void followMotor(const EctoMotor &masterMotor, bool isInverted) override;

    void enableVoltageCompensation(double nominalVoltage) override;

    void prioritizeUpdateRate() override;

    void deprioritizeUpdateRate() override;

    void setCANTimeout(int milliseconds) override;

    void outputSet(double set) override;

    bool runFaultTest() override;

    void setCurrentDraw(double currentDraw){
        this->currentDraw = currentDraw;
    }

protected:
    double position{0}, velocity{0};
    double currentDraw{0};
    double voltageOutput{0};

    double getRawAnalogPosition() const override;

    double getPotPosition() const override;

    double getPotVelocity() const override;

    double getQuadPosition() const override;

    double getQuadVelocity() const override;

    void setVoltageOutput(double voltage) override;

    void setMotorOutputByCurrent(double amps) override;

    void setOutputPercent(double value) override;

    void setPositionSetpoint(double position) override;

    void setVelocitySetpoint(double velocity) override;

    void setMotionMagicOutput(double value) override;

    void setPotAsClosedLoopSource() override;

    void setQuadAsClosedLoopSource() override;
};


#endif //BOTBUSTERS_REBIRTH_ECTOSIMULATEDMOTOR_H
