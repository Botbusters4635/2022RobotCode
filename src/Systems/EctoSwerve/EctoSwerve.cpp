//
// Created by abiel on 1/2/20.
//

#include "EctoSwerve.h"

#include "frc/smartdashboard/SmartDashboard.h"


EctoSwerve::EctoSwerve(const EctoSwerveConfig &config) : WPISubsystem("EctoSwerve") {
    /*
     * Kinematics init
     */
    kinematics = createKinematics(units::meter_t(config.length),
                                  units::meter_t(config.width));

    /*
     * Odometry init
     */
    odometry = std::make_unique<botbusters::SwerveDrivePoseEstimator<4>>(
            frc::Rotation2d(0, 0), frc::Pose2d(), *kinematics,
            wpi::array<double, 3>{0.055, 0.055, 0.055}, wpi::array<double, 1>{0.0015},
            wpi::array<double, 3>{0.0632, 0.0632, 0.0632});

    PIDConfig steerConfig, wheelConfig;
    steerConfig.p = 0.27;
    steerConfig.i = 0;
    steerConfig.d = 0.00175;

    wheelConfig.p = 0.0;
    wheelConfig.i = 0.0;
    wheelConfig.d = 0.0;

    SwerveModuleConfig moduleConfig;
    moduleConfig.gearRatio = config.gearRatio;
    moduleConfig.wheelCircumference = config.wheelCircumference;
    moduleConfig.steerPID = steerConfig;
    moduleConfig.wheelPID = wheelConfig;
    moduleConfig.steerConstraints = {
            units::radians_per_second_t(2 * M_PI * 20),
            units::radians_per_second_squared_t(
                    2.0 * M_PI * 20)};  // TODO Decide how fast it should accelerate

    moduleConfig.steerFF = {
            0.0_V, 0.0_V / 1_rad_per_s,
            0.0_V / 1_rad_per_s_sq};  // TODO Use Sysid to find constants
    moduleConfig.wheelFF = {};

    std::vector<std::string> motorPrefixes = {"front_left", "front_right",
                                              "back_left", "back_right"};

    //backup swerve offset per side fl 1.855, fr -2.845, bl 0.32, br -1.354
    std::vector<double> analogOffsets = {0.12, 2.495, 1.84, -2.98};  // fl, fr, bl, br

    for (size_t i = 0; i < modules.size(); i++) {
        const auto prefix = motorPrefixes[i];
        auto steerName = fmt::format("{}_steer", prefix);
        auto wheelName = fmt::format("{}_wheel", prefix);
        auto steerMotor = motorHandler.getMotor(steerName);
        auto wheelMotor = motorHandler.getMotor(wheelName);
        moduleConfig.analogOffset = analogOffsets[i];
        modules[i] = std::make_unique<SwerveModule>(prefix, steerMotor, wheelMotor,
                                                    moduleConfig);
    }
//    steerPIDNT = std::make_unique<NetworkTablePID>(
//            "EctoSwerve/SteerPID", moduleConfig.steerPID, [&](const auto &config) {
//                for (auto &module: modules) {
//                    module->setSteerPID(config);
//                }
//            });
//
//    wheelPIDNT = std::make_unique<NetworkTablePID>(
//            "EctoSwerve/WheelPID", moduleConfig.wheelPID, [&](const auto &config) {
//                for (auto &module: modules) {
//                    module->setWheelPID(config);
//                }
//            });

    table = ntInstance.GetTable("EctoSwerve");

    xFilter.Reset();
    yFilter.Reset();

#ifndef SIMULATION

#ifndef USE_NAVX
    /**
     * Gyro init
     */
    adis = std::make_unique<frc::ADIS16470_IMU>(
            frc::ADIS16470_IMU::IMUAxis::kZ, frc::SPI::Port::kOnboardCS0,
            frc::ADIS16470_IMU::CalibrationTime::_16s);
    // adis.Calibrate();

    adis->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
#else
    navx = std::make_unique<AHRS>(frc::SPI::kMXP, 100); //TODO @gus check if this doesn't die
    navx->Calibrate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    double startTime = frc::Timer::GetFPGATimestamp().value();

    while (navx->IsCalibrating()) {
        double timePassed = frc::Timer::GetFPGATimestamp().value() - startTime;
        if (timePassed > 16) {
            std::cout << "ERROR!!!!: NavX took too long to calibrate." << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
#endif

#endif

    thetaController = std::make_shared<SwerveThetaController>();

    zeroYaw();
    initNetworkTables();
}

void EctoSwerve::initNetworkTables() { ; }

void EctoSwerve::updateNetworkTables() {
    table->GetEntry("CurrentHeading").SetDouble(getYaw());

    double angularVel = currentVelocity.omega.value();

    odometry->SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.0632, 0.0632, 0.0632});


    if (angularVel > EctoMath::degreesToRadians(2)){
        odometry->SetVisionMeasurementStdDevs(wpi::array<double, 3>{4.832, 4.832, 3.832});
        log->error(fmt::format("VisionStdDevs set to {}", 1.832));
    }
    if(angularVel > EctoMath::degreesToRadians(180)){
        odometry->SetVisionMeasurementStdDevs(wpi::array<double, 3>{10.832, 10.832, 10.832});
        log->error(fmt::format("VisionStdDevs set to {}", 10.832));

    }

    bool slowVel = currentVelocity.omega.value() < EctoMath::degreesToRadians(15);
    bool notStill = currentVelocity.omega.value() > EctoMath::degreesToRadians(0.8);
    if(slowVel && notStill){
//        odometry->SetVisionMeasurementStdDevs(wpi::array<double, 3>{1.532, 1.532, 1.532});
    }

    auto currentPose = getPose();

    table->GetEntry("Odometry/Velocity/dX").SetDouble(currentVelocity.vx.value());
    table->GetEntry("Odometry/Velocity/dY").SetDouble(currentVelocity.vy.value());
    table->GetEntry("Odometry/Velocity/dTheta")
            .SetDouble(currentVelocity.omega.value());



    table->GetEntry("Odometry/X").SetDouble(currentPose.X().value());
    table->GetEntry("Odometry/Y").SetDouble(currentPose.Y().value());
    table->GetEntry("Odometry/Heading")
            .SetDouble(currentPose.Rotation().Radians().value());
    table->GetEntry("Odometry/Point")
            .SetString(fmt::format("({},{})", currentPose.X().value(),
                                   currentPose.Y().value()));
}

void EctoSwerve::setVelocity(const frc::ChassisSpeeds &target) {
    auto states = SwerveState(kinematics->ToSwerveModuleStates(target));
    SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState,
                                0);
    setModules(states,
               MotorControlMode::Velocity);  // TODO normalize to maximum velocity
    lastState = states;
}

void EctoSwerve::setPercent(const frc::ChassisSpeeds &target,
                            const Point2D &point) {
    frc::Translation2d cor(units::meter_t(point.getX()),
                           units::meter_t(point.getY()));

    auto modTarget = target;

    if(thetaController->isEnabled()){
        modTarget.omega = units::radians_per_second_t(thetaController->getTarget().value() / 12.0);
        lastSetTime = frc::Timer::GetFPGATimestamp();
    }

    velMagnitude = frc::Velocity2d(target.vx, target.vy).Norm().value();

    auto wpiStates = kinematics->ToSwerveModuleStates(modTarget, cor);
    kinematics->DesaturateWheelSpeeds(&wpiStates, units::meters_per_second_t(1));

    auto states = SwerveState(wpiStates);
    SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState,
                                0);

    setModules(states, MotorControlMode::Percent);
    lastState = states;
}

void EctoSwerve::setVoltage(const frc::ChassisSpeeds &target) {
    auto modTarget = target;

    if(thetaController->isEnabled()){
        modTarget.omega = units::radians_per_second_t(thetaController->getTarget().value());
        lastSetTime = frc::Timer::GetFPGATimestamp();
    }

    auto wpiStates = kinematics->ToSwerveModuleStates(modTarget);
    kinematics->DesaturateWheelSpeeds(&wpiStates,
                                      units::meters_per_second_t(12.0));
    auto states = SwerveState(wpiStates);
    SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState,
                                0);

    setModules(states, MotorControlMode::Voltage);
    lastState = states;
}

SwerveState EctoSwerve::getMotorStates() const {
    SwerveState values;

    for (int i = 0; i < 4; i++) {
        *values.wheels[i] = modules[i]->getState();
    }

    return values;
}

void EctoSwerve::setModules(const SwerveState &rawSetpoint,
                            MotorControlMode controlMode) {
    auto state = getMotorStates();
    auto dt = frc::Timer::GetFPGATimestamp().value() - lastRunTime;

    auto setpoint = SwerveState::optimizeValues(state, rawSetpoint, lastState,
                                                lastSetpoint, dt);

    for (int i = 0; i < 4; i++) {
        auto wheel = setpoint.wheels[i];
        modules[i]->setState(wheel->wheelAngle, wheel->wheelVelocity, controlMode);
    }

    lastSetpoint = setpoint;
    lastState = state;
    lastRunTime = frc::Timer::GetFPGATimestamp().value();
}

void EctoSwerve::robotInit() {
    ;
}

void EctoSwerve::robotUpdate() {
    updateNetworkTables();

    auto motorStates = getMotorStates();
    updateOdometry(motorStates);

    /**
     * Only theta controller is being used, call set voltage with 0 vx,vy
     * so that theta can be set
     */
    if(thetaController->isEnabled() and frc::Timer::GetFPGATimestamp() - lastSetTime > 25_ms){
        frc::ChassisSpeeds vel;
        setVoltage(vel);
    }

    if(thetaController->isEnabled() != lastThetaControllerState and !thetaController->isEnabled()){
        //Manually set velocity to 0 after theta controller is disabled
        std::cout << "Theta controller disabled" << std::endl;
        frc::ChassisSpeeds vel;
        setVoltage(vel);
    }

    lastThetaControllerState = thetaController->isEnabled();

    auto state = getPose();
    if(std::isnan(state.X().value()) or std::isnan(state.Y().value())){
        log->error("Odometry nan, resetting position!");
        pe_NMutex.lock();
        pe_MMutex.lock();
        pe_NMutex.unlock();
        odometry->ResetPosition(
                {},
                {});
        pe_MMutex.unlock();
    }
}

void EctoSwerve::zeroYaw() {
    /**
     * NavX Reset
     */
    headingZero = getYaw(false);
    rollZero = getRoll(false);
    pitchZero = getPitch(false);
}

void EctoSwerve::setYaw(double yaw) {
#ifndef SIMULATION
    headingZero = getYaw(false) - yaw;
#else
    simYaw = yaw;
#endif
}

double EctoSwerve::getRawYaw() const {
#ifdef USE_NAVX
#ifndef SIMULATION
    return -navx->GetAngle() * (M_PI) / 180.0;
#endif
#endif
    return 0; //TODO
}

double EctoSwerve::getYaw(bool useZero) const {
    double yaw = 0;

#ifndef SIMULATION
#ifndef USE_NAVX
    yaw = adis->GetAngle().value() * (M_PI / 180.0);
    yaw = EctoMath::wrapAngle(yaw);
#else
    yaw = -navx->GetYaw() * (M_PI / 180.0);
    yaw = EctoMath::wrapAngle(yaw);
#endif

#else
    yaw = EctoMath::wrapAngle(simYaw);
#endif

    if (useZero) {
        yaw -= headingZero;
    }

    yaw = EctoMath::wrapAngle(yaw);

    return yaw;
}

double EctoSwerve::getRoll(bool useZero) const {
    double roll = 0;
#ifndef SIMULATION

    roll = navx->GetRoll() * (M_PI / 180.0);

#endif

    roll = EctoMath::wrapAngle(roll);
    if (useZero) { roll -= rollZero; }
    roll = EctoMath::wrapAngle(roll);
    return roll;
}

double EctoSwerve::getPitch(bool useZero) const {
    double pitch = 0;
#ifndef SIMULATION

    pitch = navx->GetPitch() * (M_PI / 180.0);

#endif

    pitch = EctoMath::wrapAngle(pitch);
    if (useZero) { pitch -= pitchZero; }
    pitch = EctoMath::wrapAngle(pitch);
    return pitch;
}

frc::Pose2d EctoSwerve::getPose() {
    pe_NMutex.lock();
    std::lock_guard<std::mutex> lg(pe_MMutex);
    pe_NMutex.unlock();

    auto pose = odometry->GetEstimatedPosition();
//
//    auto x = units::meter_t(xFilter.Calculate(pose.X().value()));
//    auto y = units::meter_t(yFilter.Calculate(pose.Y().value()));
//
//
//    return {x, y, pose.Rotation()};
    return pose;
}

void EctoSwerve::addVisionPoseMeasurement(const frc::Pose2d visionPose, double timeStamp) {
    //Low priority lock
    pe_LMutex.lock();
    pe_NMutex.lock();
    pe_MMutex.lock();
    pe_NMutex.unlock();
    odometry->AddVisionMeasurement(visionPose, units::second_t(timeStamp));
    pe_MMutex.unlock();
    pe_LMutex.unlock();
}


void EctoSwerve::resetOdometry(const frc::Pose2d newPose) {
    setYaw(newPose.Rotation().Radians().value());
    pe_NMutex.lock();
    pe_MMutex.lock();
    pe_NMutex.unlock();
    odometry->ResetPosition(
            newPose,
            newPose.Rotation());  // TODO Maybe use odometry to return the heading, it
    // can handle setting the heading on its own
    pe_MMutex.unlock();
}

frc::Velocity2d EctoSwerve::getVelocity() const {
    return {currentVelocity.vx, currentVelocity.vy};
}

frc::Rotation2d EctoSwerve::getRotation()  {
    return getPose().Rotation();
}

frc::ChassisSpeeds EctoSwerve::getChassisSpeeds(){
    return currentVelocity;
}

void EctoSwerve::updateOdometry(const SwerveState &motorStates) {
    wpi::array<frc::SwerveModuleState, 4> moduleStates = motorStates.toWPI();
    currentVelocity = kinematics->ToChassisSpeeds(moduleStates);

    pe_NMutex.lock();
    pe_MMutex.lock();
    pe_NMutex.unlock();
    odometry->UpdateWithTime(frc::Timer::GetFPGATimestamp() ,Rotation2d(units::angle::radian_t(getYaw())),
                     moduleStates);
    pe_MMutex.unlock();
}

std::vector<std::shared_ptr<ChecklistItem>> EctoSwerve::createTests() {
    return {};
}

units::volt_t EctoSwerve::calculateRotationFF(const units::radians_per_second_t &velocity) const {
    return rotationFF.Calculate(velocity);
}