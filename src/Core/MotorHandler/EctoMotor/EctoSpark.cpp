//
// Created by hiram on 8/08/19.
//
#include "EctoSpark.h"
#include "EctoMotor.h"
#include "Math/EctoMath.h"
#include <string_view>

static std::string_view faultIdToString(rev::CANSparkMax::FaultID fault){
    std::vector<std::string_view> faults = {
            "Brownout", "Overcurrent", "IWDTReset", "MotorFault", "SensorFault",
            "Stall", "EEPROMCRC", "CANTX", "CANRX", "HasReset", "DRVFault",
            "OtherFault", "SoftLimitFwd", "SoftLimitRev", "HardLimitFwd", "HardLimitRev"};

    auto fault_code = static_cast<int>(fault);

    if(fault_code > 0 && fault_code < 16)
        return faults[fault_code];

    return "InvalidFault";
}

static std::vector<std::string_view> decodeFaults(uint16_t faults){
    std::vector<std::string_view> out;
    for(int i = 0; i < 12; i++){
        bool faultbit = ((faults) & (1 << i));
        if(faultbit) out.emplace_back(faultIdToString(static_cast<rev::CANSparkMax::FaultID>(i)));
    }

    return out;
}

EctoSpark::EctoSpark(int id, const std::string &name, bool brushed, bool doTests) : EctoMotor(id, name,
                                                                                brushed ? EctoMotorType::SparkMaxBrushed
                                                                                        : EctoMotorType::SparkMax, doTests) {
	if (brushed) {
		sparkBase = std::make_unique<rev::CANSparkMax>(id, rev::CANSparkMax::MotorType::kBrushed);
	} else {
		sparkBase = std::make_unique<rev::CANSparkMax>(id, rev::CANSparkMax::MotorType::kBrushless);
	}
	sparkBase->RestoreFactoryDefaults();
	
	pidControllerBase = std::make_unique<rev::SparkMaxPIDController>(sparkBase->GetPIDController());
	
	if (brushed) {
		encoderSensor = std::make_unique<rev::SparkMaxRelativeEncoder>(
				sparkBase->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kQuadrature));
	} else {
		encoderSensor = std::make_unique<rev::SparkMaxRelativeEncoder>(sparkBase->GetEncoder());
	}
	
	encoderSensor->SetPosition(0);
	analogSensor = std::make_unique<rev::SparkMaxAnalogSensor>(sparkBase->GetAnalog(rev::SparkMaxAnalogSensor::Mode::kAbsolute));

	sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10);
	sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 20);
    sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 20);
    sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 50);

	forwardLimitSwitch = std::make_unique<rev::SparkMaxLimitSwitch>(
			sparkBase->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen));
	reverseLimitSwitch = std::make_unique<rev::SparkMaxLimitSwitch>(
			sparkBase->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen));

}

void EctoSpark::setClosedLoopOutputRange(double minimum, double maximum) {
	pidControllerBase->SetOutputRange(minimum, maximum);
}

void EctoSpark::invertMotor(bool state) {
    std::lock_guard<std::mutex> lg(mutex);
	sparkBase->SetInverted(state);
}

bool EctoSpark::isMotorInverted() const {
    std::lock_guard<std::mutex> lg(mutex);
	return sparkBase->GetInverted();
}

bool EctoSpark::isSensorInverted() const {
    std::lock_guard<std::mutex> lg(mutex);
	switch (feedbackMode) {
		case MotorFeedbackMode::None:
			return false;
		case MotorFeedbackMode::QuadEncoder:
			return encoderSensor->GetInverted();
		case MotorFeedbackMode::Potentiometer:
			return analogSensor->GetInverted();
		default:
			return false;
	}
}

void EctoSpark::setPIDConfig(const PIDConfig &pidConfig, int profileSlot) {
	//TODO use profile slot
    std::lock_guard<std::mutex> lg(mutex);
	pidControllerBase->SetP(pidConfig.p);
	pidControllerBase->SetI(pidConfig.i);
	pidControllerBase->SetD(pidConfig.d);
	pidControllerBase->SetFF(pidConfig.f);
	pidControllerBase->SetOutputRange(-1, 1);
}

void EctoSpark::enableCurrentLimit(bool state) {
    std::lock_guard<std::mutex> lg(mutex);
	//No way to disable the current limit (probably not a good idea to do so)
	if (state) {
		sparkBase->SetSmartCurrentLimit((unsigned int) std::abs(currentLimit));
	} else {
		log->warn("Current limit for spark motor: {} disabled", getName());
		sparkBase->SetSmartCurrentLimit((unsigned int) std::abs(maximumCurrentLimit));
	}
}

void EctoSpark::setMotorCurrentLimit(double current) {
    std::lock_guard<std::mutex> lg(mutex);
    sparkBase->SetSmartCurrentLimit((unsigned int) std::abs(current));
	//sparkBase->SetSecondaryCurrentLimit(current);
}

void EctoSpark::setMotorOutputByCurrent(double value) {
    std::lock_guard<std::mutex> lg(mutex);
    pidControllerBase->SetReference(value,
                                    rev::CANSparkMax::ControlType::kCurrent, 0, currentFeedForward, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}

void EctoSpark::setClosedLoopRampRate(double rampRate) {
    std::lock_guard<std::mutex> lg(mutex);
    sparkBase->SetClosedLoopRampRate(rampRate);
}

void EctoSpark::setOpenLoopRampRate(double rampRate) {
    std::lock_guard<std::mutex> lg(mutex);
    sparkBase->SetOpenLoopRampRate(rampRate);
}

void EctoSpark::enableBrakingOnIdle(bool state) {
    std::lock_guard<std::mutex> lg(mutex);
    sparkBase->SetIdleMode(state ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
}


void EctoSpark::invertSensor(bool state) {
    std::lock_guard<std::mutex> lg(mutex);
    if (feedbackMode == MotorFeedbackMode::None || feedbackMode == MotorFeedbackMode::QuadEncoder) {
		throw std::runtime_error("Cannot invertMotor sensor as it is not defined or its a brushless encoder");
	}
	
	analogSensor->SetInverted(state);
}

double EctoSpark::getOutputPercent() const {
    std::lock_guard<std::mutex> lg(mutex);
	return sparkBase->GetAppliedOutput();
}

void EctoSpark::setOutputPercent(double value) {
    std::lock_guard<std::mutex> lg(mutex);
	sparkBase->Set(value);
}

void EctoSpark::setSensorPosition(double position) {
    std::lock_guard<std::mutex> lg(mutex);
	encoderSensor->SetPosition(position / (2.0 * M_PI));
}

void EctoSpark::setVoltageOutput(double voltage) {
    std::lock_guard<std::mutex> lg(mutex);
	pidControllerBase->SetReference(voltage,
                                    rev::CANSparkMaxLowLevel::ControlType::kVoltage, 0, currentFeedForward, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}

void EctoSpark::setVelocitySetpoint(double velocity) {
	velocity = velocity / (2.0 * M_PI) * 60;
	pidControllerBase->SetReference(velocity,
	                                rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0, currentFeedForward, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}

void EctoSpark::setEncoderCodesPerRev(int codes) {
    std::lock_guard<std::mutex> lg(mutex);
    if (codes == 0) {
		log->warn("Encoder codes set to zero in motor with name: {}, config not applied", codes);
		return;
	}
	
	encoderSensor->SetPositionConversionFactor(1);
	encoderSensor->SetVelocityConversionFactor(1);
	
	//sparkBase->GetEncoder().SetPositionConversionFactor(42.0 / codes);
	//sparkBase->GetEncoder().SetVelocityConversionFactor(42.0 / codes);
	
}

int EctoSpark::getEncoderCodesPerRev() const {
    std::lock_guard<std::mutex> lg(mutex);
    return encoderSensor->GetPositionConversionFactor() * 42;
}


double EctoSpark::getTemperature() const {
    std::lock_guard<std::mutex> lg(mutex);
    return sparkBase->GetMotorTemperature();
}

double EctoSpark::getCurrent() const {
    std::lock_guard<std::mutex> lg(mutex);
    return 0;
	return sparkBase->GetOutputCurrent();
}

double EctoSpark::getVoltage() const {
    std::lock_guard<std::mutex> lg(mutex);
	return sparkBase->GetBusVoltage();
}

void EctoSpark::setArbitraryFeedForward(double feedForward) {
    std::lock_guard<std::mutex> lg(mutex);
    if (std::abs(feedForward) > 32.0) {
		log->error("Invalid feedforward: {} given to EctoSpark with id: {}", feedForward, id);
		return;
	}
	
	this->currentFeedForward = feedForward;
}

void EctoSpark::disable() {
    std::lock_guard<std::mutex> lg(mutex);
    sparkBase->Disable();
	disabled = true;
}

bool EctoSpark::isDisabled() const {
    std::lock_guard<std::mutex> lg(mutex);
    return disabled;
}

void EctoSpark::factoryReset() {
    std::lock_guard<std::mutex> lg(mutex);
    sparkBase->RestoreFactoryDefaults(false);
}

void EctoSpark::enableLimitSwitches(bool state) {
	forwardLimitSwitch->EnableLimitSwitch(state);
	reverseLimitSwitch->EnableLimitSwitch(state);
}

void EctoSpark::setForwardSoftLimit(double radians) {
	sparkBase->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ((radians / (2.0 * M_PI))));
}

void EctoSpark::enableForwardSoftLimit(bool state) {
	sparkBase->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, state);
}

void EctoSpark::setReverseSoftLimit(double radians) {
	sparkBase->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, (radians / (2.0 * M_PI)));
}

void EctoSpark::enableReverseSoftLimit(bool state) {
	sparkBase->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, state);
}

void EctoSpark::configureMotionMagicVelocity(double velocity) {
	//rad/s to rpm
	pidControllerBase->SetSmartMotionMaxVelocity((velocity / (M_PI * 2.0)) * 60.0);
}

void EctoSpark::configureMotionMagicAcceleration(double acceleration) {
	//rad/s^2 to rpm/min
	pidControllerBase->SetSmartMotionMaxAccel((acceleration / (M_PI * 2.0)) * std::pow(60.0, 2.0));
}

void EctoSpark::configureMotionMagicSCurve(double sCurve) {
	log->error("Motion magic SCurves are still not implemented in Spark Max API");
}

void EctoSpark::setMotionMagicOutput(double value) {
	pidControllerBase->SetReference(value / (2.0 * M_PI),
                                    rev::CANSparkMaxLowLevel::ControlType::kSmartMotion, 0, currentFeedForward, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}


void EctoSpark::setAnalogPositionConversionFactor(double conversionFactor) {
	analogSensor->SetPositionConversionFactor(conversionFactor);
}

void EctoSpark::setAnalogVelocityConversionFactor(double conversionFactor) {
	analogSensor->SetVelocityConversionFactor(conversionFactor);
}

void EctoSpark::setPositionSetpoint(double position) {
	pidControllerBase->SetReference(position / (2.0 * M_PI),
                                    rev::CANSparkMaxLowLevel::ControlType::kPosition, 0, currentFeedForward, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}

#include <frc/Timer.h>

double EctoSpark::getPotPosition() const {
    std::lock_guard<std::mutex> lg(mutex);
	double pos = analogSensor->GetPosition();
	pos -= analogOffset;
	pos = EctoMath::wrapAngle(pos);
	
	return pos;
}

double EctoSpark::getRawAnalogPosition() const {
    std::lock_guard<std::mutex> lg(mutex);
	return analogSensor->GetPosition();
}

double EctoSpark::getPotVelocity() const {
    std::lock_guard<std::mutex> lg(mutex);
	return analogSensor->GetVelocity();
}

double EctoSpark::getQuadPosition() const {
    std::lock_guard<std::mutex> lg(mutex);
	return encoderSensor->GetPosition() * (2.0 * M_PI); //return radians
}

double EctoSpark::getQuadVelocity() const {
    std::lock_guard<std::mutex> lg(mutex);
	return (encoderSensor->GetVelocity() / 60.0) * (2.0 * M_PI); //returns radians per second
}

void EctoSpark::setPotAsClosedLoopSource() {
	pidControllerBase->SetFeedbackDevice(*analogSensor);
}

void EctoSpark::setQuadAsClosedLoopSource() {
	pidControllerBase->SetFeedbackDevice(*encoderSensor);
}

void EctoSpark::setAnalogSensorOffset(double analogOffset) {
	EctoSpark::analogOffset = analogOffset;
}

std::string EctoSpark::getFirmwareVersion() const {
	return sparkBase->GetFirmwareString();
}

void EctoSpark::setLimitSwitchPolarity(bool normallyClosed) {
	if (normallyClosed) {
		forwardLimitSwitch = std::make_unique<rev::SparkMaxLimitSwitch>(
				sparkBase->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed));
		reverseLimitSwitch = std::make_unique<rev::SparkMaxLimitSwitch>(
				sparkBase->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed));
	} else {
		forwardLimitSwitch = std::make_unique<rev::SparkMaxLimitSwitch>(
				sparkBase->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen));
		reverseLimitSwitch = std::make_unique<rev::SparkMaxLimitSwitch>(
				sparkBase->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen));
	}
	
}

bool EctoSpark::getReverseLimitSwitch() const {
	return reverseLimitSwitch->Get();
}

bool EctoSpark::getForwardLimitSwitch() const {
	return forwardLimitSwitch->Get();
}

void EctoSpark::followMotor(const EctoMotor &masterMotor, bool inverted) {
    std::lock_guard<std::mutex> lg(mutex);
	switch (masterMotor.getMotorType()) {
		case EctoMotorType::TalonSRX:
			sparkBase->Follow(rev::CANSparkMax::kFollowerPhoenix, masterMotor.getId(), inverted);
			break;
		
		case EctoMotorType::SparkMax:
			sparkBase->Follow(rev::CANSparkMax::kFollowerSparkMax, masterMotor.getId(), inverted);
			break;
		
		default:
			log->error("Motor: {} tried to follow invalid motor: {} with invalid type", this->getId(),
			           masterMotor.getId());
			return;
	}
}

void EctoSpark::enableVoltageCompensation(double nominalVoltage) {
	sparkBase->EnableVoltageCompensation(nominalVoltage);
}

void EctoSpark::prioritizeUpdateRate() {
	sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10);
	sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10);
    sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10);
}

void EctoSpark::deprioritizeUpdateRate(){
	sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 25);
	sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 150);
    sparkBase->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 150);
}


bool EctoSpark::runFaultTest() {
    auto faults = decodeFaults(sparkBase->GetFaults());
    auto stickyFaults = decodeFaults(sparkBase->GetStickyFaults());

    return (faults.empty() && stickyFaults.empty());
}

void EctoSpark::setCANTimeout(int milliseconds) {
    sparkBase->SetCANTimeout(milliseconds);
}