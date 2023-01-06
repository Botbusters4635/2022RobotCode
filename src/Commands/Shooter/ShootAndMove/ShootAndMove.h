//
// Created by abiel on 4/9/22.
//

#ifndef BOTBUSTERS_REBIRTH_SHOOTANDMOVE_H
#define BOTBUSTERS_REBIRTH_SHOOTANDMOVE_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Math/InterpolatingTable/InterpolatingTable.h"
#include <frc/filter/LinearFilter.h>

#include "Systems/Generic/Shooter.h"
#include "Systems/Feeder/Feeder.h"
#include "Systems/PIDTurret/PIDTurret.h"
#include "Core/VisionManager/VisionManager.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "Control/LQR/SwerveRotationLQR.h"

struct ShootAndMoveConfig {
    InterpolatingTable hoodTable, shooterTable;
    InterpolatingTable tofTable;
};

class ShootAndMove : public frc2::CommandHelper<frc2::CommandBase, ShootAndMove> {
public:
    ShootAndMove(const std::shared_ptr<Shooter> &shooter,
                 const std::shared_ptr<EctoSwerve> &swerve,
                 const std::shared_ptr<Feeder> &feeder,
                 const std::shared_ptr<VisionManager> &vision,
                 const std::shared_ptr<PIDTurret> &turret,
                 const ShootAndMoveConfig &config);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::shared_ptr<Shooter> shooter;
    std::shared_ptr<VisionManager> vision;
    std::shared_ptr<SwerveThetaController> swerveThetaController;
    std::shared_ptr<EctoSwerve> swerve;
    std::shared_ptr<Feeder> feeder;
    std::shared_ptr<PIDTurret> turret;

    void doPreload();

    ShootAndMoveConfig config;

    frc::ProfiledPIDController<units::radian> visionAnglePID{0.3151, 0.027, 0.0, {10_rad_per_s,
                                                                               10_rad_per_s_sq}}; //the need for unified configs

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;
    frc::LinearFilter<double> xFilter = frc::LinearFilter<double>::SinglePoleIIR(0.0115, 20_ms);
    frc::LinearFilter<double> yFilter = frc::LinearFilter<double>::SinglePoleIIR(0.0115, 20_ms);
    units::second_t allowShootTime;

    frc::LinearFilter<double> visionFilter = frc::LinearFilter<double>::SinglePoleIIR(0.15, 20_ms);
    frc::LinearFilter<double> movingAvFilter = frc::LinearFilter<double>::MovingAverage(5);

    int ballCount{0};

    double turretError{0};

    bool initialWait;
    double initialTime;

};

#endif //BOTBUSTERS_REBIRTH_SHOOTANDMOVE_H
