//
// Created by cc on 19/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_FEEDER_H
#define BOTBUSTERS_REBIRTH_FEEDER_H

#include "Core/EctoModule/WPISubsystem.h"
#include "Core/MotorHandler/EctoMotor/EctoMotor.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/DigitalInput.h>
#include <frc/Counter.h>
#include <frc/DigitalGlitchFilter.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>



struct FeederConfig{
    PIDConfig pidConfig;
    std::shared_ptr<EctoMotor> feederMotor;

    bool isInverted;

    double rampRate ;

    double currentLimit;

    bool enableBreakingOnIdle;

    double gearReduction;

    frc::SimpleMotorFeedforward<units::radian> ff {0_V, 0_V / 1_rad_per_s, 0_V / 1_rad_per_s_sq};

    std::vector<double> limitSwitchIds;
    std::vector<bool> invertedSwitches;
};

class Feeder : public WPISubsystem{
public:
    explicit Feeder(const FeederConfig &config);

    void robotInit() override;
    void robotUpdate() override;

    void setFeederVol(double setVoltage);

    bool getIntakeLimit();

    bool getMiddleLimit();

    bool getFeederLimit();

    int getBallsShot();

    void resetBallsShot();


private:
    FeederConfig config;
    std::shared_ptr<EctoMotor> feederMotor;

    std::unique_ptr<frc2::PIDController> wpiPID;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;
    nt::NetworkTableEntry feederVel, limitTopState, limitBottomState, limitMiddleState, current, ballsShot, feederAppliedVoltage;

    frc::DigitalInput *feederLimit, *middleLimit, *intakeLimit;
#ifndef SIMULATION
    std::unique_ptr<frc::Counter> counter;
    frc::DigitalGlitchFilter digitalGlitchFilter;
#endif
};

#endif //BOTBUSTERS_REBIRTH_FEEDER_H
