#include "SwerveModule.h"

#include <utility>
#include "Math/EctoMath.h"
#include <frc/Timer.h>
#include <frc/DriverStation.h>

SwerveModule::SwerveModule(const std::string &name, const std::shared_ptr<EctoMotor> &steerMotor,
                           const std::shared_ptr<EctoMotor> &wheelMotor, const SwerveModuleConfig &config) {
	moduleName = name;
	
	this->steerMotor = steerMotor;
	this->wheelMotor = wheelMotor;
	
	wheelMotor->setPIDConfig(config.wheelPID);
	
	steerMotor->enableBrakingOnIdle(true);
	wheelMotor->enableBrakingOnIdle(true);
	
	steerMotor->setMotorCurrentLimit(config.steerCurrentLimit);
	steerMotor->enableCurrentLimit(true);
	
	wheelMotor->setMotorCurrentLimit(config.wheelCurrentLimit);
	wheelMotor->enableCurrentLimit(true);
	
	steerMotor->setAnalogPositionConversionFactor(2 * M_PI / 3.33333333); //Potentiometer v/rot
	steerMotor->setAnalogSensorOffset(config.analogOffset);
	analogOffset = config.analogOffset;
	
	steerMotor->setFeedbackMode(MotorFeedbackMode::Potentiometer);
	steerMotor->setOpenLoopRampRate(0.115);
	
	wheelMotor->invertMotor(false);
	steerMotor->invertMotor(false);
	steerMotor->invertSensor(true);
	
	wheelMotor->setClosedLoopRampRate(0.05);
	wheelMotor->setOpenLoopRampRate(0.05);

    steerMotor->enableLimitSwitches(false);

    steerMotor->burnFlash();
    wheelMotor->burnFlash();
	
	wheelCircumference = config.wheelCircumference;
	gearRatio = config.gearRatio;
	
	steerController = std::make_unique<frc2::PIDController>(
			config.steerPID.p, config.steerPID.i, config.steerPID.d
	);
	
	wheelController = std::make_unique<frc2::PIDController>(
			config.wheelPID.p, config.wheelPID.i, config.wheelPID.d
	);

	steerController->EnableContinuousInput(-M_PI, M_PI);
	
	
	analogFilter = std::make_unique<frc::LinearFilter<double>>(
			frc::LinearFilter<double>::SinglePoleIIR(0.05,
			                                         units::millisecond_t(20))
	);

	
	pidNotifier = std::make_unique<frc::Notifier>([this, config] {
		//updateNT();
		const auto currentState = this->getState();
        stateMutex.lock();
        const auto setpoint = this->wheelSetpoint;

		double pos = currentState.wheelAngle;


		if (config.enableAnalogFilter) {
			pos = analogFilter->Calculate(pos);
		}

		double out = this->steerController->Calculate(pos);
		

//        double ff = config.steerFF.Calculate(this->steerController->GetSetpoint().velocity).value();
//        out += ff;

		this->steerMotor->set(out, MotorControlMode::Percent);

		if(this->controlMode == MotorControlMode::Velocity){
			double wheelOut = this->wheelController->Calculate(currentState.wheelVelocity);
			wheelOut += config.wheelFF.Calculate(units::meters_per_second_t(setpoint)).value();
			this->wheelMotor->set(wheelOut, MotorControlMode::Voltage);
		}
        stateMutex.unlock();
	});
	
	table = nt::NetworkTableInstance::GetDefault().GetTable(fmt::format("SwerveModule/{}", moduleName));
	velocityEntry = table->GetEntry("Velocity");
	angleEntry = table->GetEntry("Angle");
	wheelSetpointEntry = table->GetEntry("WheelSetpoint");
	steerSetpointEntry = table->GetEntry("SteerSetpoint");
	controlModeEntry = table->GetEntry("ControlMode");
	encoderState = table->GetEntry("EncoderPosition");
	rawAnalog = table->GetEntry("RawAnalog");
    ffEntry = table->GetEntry("FF");
    steerCurrentEntry = table->GetEntry("SteerCurrent");
    wheelCurrentEntry = table->GetEntry("WheelCurrent");

	
	ntNotifier = std::make_unique<frc::Notifier>([this] {
		this->updateNT();
	});
	
	pidNotifier->StartPeriodic(20_ms);
	 ntNotifier->StartPeriodic(ntUpdateRate);
}

void SwerveModule::setState(double angle, double motorSetpoint, MotorControlMode wheelControlMode) {
    std::lock_guard<std::mutex> lg(stateMutex);
	steerController->SetSetpoint(angle);
	
	if(wheelControlMode == MotorControlMode::Velocity){
		wheelController->SetSetpoint(motorSetpoint);
	}else{
		wheelMotor->set(motorSetpoint, wheelControlMode);
	}
	
	controlMode = wheelControlMode;
	wheelSetpoint = motorSetpoint;
	steerSetpoint = angle;
}

SwerveWheel SwerveModule::getState() const {
    std::lock_guard<std::mutex> lg(stateMutex);
    SwerveWheel out;

    out.wheelPosition = (wheelMotor->getPosition() / (2.0 * M_PI)) / gearRatio * wheelCircumference;

    const auto dt = frc::Timer::GetFPGATimestamp() - lastStateUpdate;

    if(lastStateUpdate == 0_s){
        //Initial state, velocity is 0
        out.wheelVelocity = 0;
    } else{
        out.wheelVelocity = (out.wheelPosition - lastWheelPosition) / dt.value();
    }

    if(!usePosDeltaVelocity){
        out.wheelVelocity = (wheelMotor->getVelocity() / (2.0 * M_PI)) / gearRatio * wheelCircumference;
    }

	lastStateUpdate = frc::Timer::GetFPGATimestamp();
	out.wheelAngle = steerMotor->getPosition();
	out.wheelAngularVelocity = steerMotor->getVelocity();
	
	return out;
}

void SwerveModule::updateNT() {
	auto state = getState();
	
	velocityEntry.SetDouble(state.wheelVelocity);
	angleEntry.SetDouble(state.wheelAngle);
	wheelSetpointEntry.SetDouble(wheelSetpoint);
	steerSetpointEntry.SetDouble(steerSetpoint);
	controlModeEntry.SetString(toString(controlMode));
	encoderState.SetDouble(wheelMotor->getPosition());
    steerCurrentEntry.SetDouble(steerMotor->getCurrent());
    wheelCurrentEntry.SetDouble(wheelMotor->getCurrent());
	
	auto analogPos = steerMotor->getPosition();
	analogPos += analogOffset;
	analogPos = EctoMath::wrapAngle(analogPos);
	rawAnalog.SetDouble(analogPos);
}

