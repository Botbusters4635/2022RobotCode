#include <frc/RobotBase.h>
#include <frc/UidSetter.h>
#include <frc/Processes.h>
#include <fmt/format.h>
#include <fmt/os.h>
#include <frc/livewindow/LiveWindow.h>
#include "Robots/2022/ToyotaViolinPlayingRobot.h"
#include "Robots/2022/Characterization/SwerveCharacterizationRobot.h"
#include "Robots/2022/Characterization/SwerveCharacterizationRotation.h"
#include "Robots/2022/Characterization/TurretCharacterizationRobot.h"
#include "Robots/2022/Characterization/ElevatorCharacterizationRobot.h"
#include "Robots/2022/Characterization/ShooterCharacterizationRobot.h"
#include "Robots/2022/TurretTesting.h"
#include "Robots/2022/ElevatorTesting.h"

/**
 * Botbusters Rebirth 2021
 *
 * Created by:
 * Gustavo Ramos Ramos
 * Andres Alarcon Navarro
 * Abiel Fernandez
 * Hiram Muñoz
 * Alberto Jahuey
 * Karen Rodriguez
 *
 *                     __---__
 *                  _-       _--______
 *             __--( /     \ )XXXXXXXXXXXXX_
 *           --XXX(   O   O  )XXXXXXXXXXXXXXX-
 *          /XXX(       U     )        XXXXXXX\
 *        /XXXXX(              )--_  XXXXXXXXXXX\
 *       /XXXXX/ (      O     )   XXXXXX   \XXXXX\
 *       XXXXX/   /            XXXXXX   \__ \XXXXX----
 *       XXXXXX__/          XXXXXX         \__----  -
-*--___  XXX__/          XXXXXX      \__         ---
 * --  --__/   ___/\  XXXXXX            /  ___---=
 *   -_    ___/    XXXXXX              '--- XXXXXX
 *     --\/XXX\ XXXXXX                      /XXXXX
 *       \XXXXXXXXX                        /XXXXX/
 *        \XXXXXX                        _/XXXXX/
 *          \XXXXX--__/              __-- XXXX/
 *           --XXXXXXX---------------  XXXXX--
 *              \XXXXXXXXXXXXXXXXXXXXXXXX-
 *                --XXXXXXXXXXXXXXXXXX-
 *          * * * * * who ya gonna call? * * * * *
 *
 *    ______       _   _               _
 *    | ___ \     | | | |             | |
 *    | |_/ / ___ | |_| |__  _   _ ___| |_ ___ _ __ ___
 *    | ___ \/ _ \| __| '_ \| | | / __| __/ _ \ '__/ __|
 *    | |_/ / (_) | |_| |_) | |_| \__ \ ||  __/ |  \__ \
 *    \____/ \___/ \__|_.__/ \__,_|___/\__\___|_|  |___/
 *
 */

#ifndef RUNNING_FRC_TESTS

#ifndef SIMULATION

void SetRTRuntimeLimit() {
    if constexpr (!frc::RobotBase::IsSimulation()) {
        frc::UidSetter uidSetter{0};

        auto setting = fmt::output_file("/proc/sys/kernel/sched_rt_runtime_us");
        setting.print("950000\n");
    }
}

void StopCrond() {
    if constexpr (!frc::RobotBase::IsSimulation()) {
        frc::UidSetter uidSetter{0};

        int status = std::system("/etc/init.d/crond stop");
        if (status != 0) {
            throw std::runtime_error(
                    fmt::format("Failed to stop crond ({})", status));
        }
    }
}

#endif

int main() {
    std::shared_ptr<spdlog::logger> log = spdlog::stdout_color_mt("Main");
    spdlog::set_level(spdlog::level::info);

#ifndef SIMULATION
    const bool useRTPriority = false;
    // Give FRC_NetCommDaemon higher priority so it's not preempted by user code
    // during high CPU utilization
    constexpr int kPrioNetCommDaemon = 35;

    // Give HAL Notifier thread highest priority so robot code Notifiers are
    // accurately scheduled
    constexpr int kPrioHALNotifierThread = 40;

    if (useRTPriority) {
        {
            SetRTRuntimeLimit();

            frc::UidSetter uidSetter{0};
            if (!frc::Notifier::SetHALThreadPriority(true,
                                                     kPrioHALNotifierThread)) {
                throw std::runtime_error(
                        fmt::format("Giving HAL Notifier RT priority {} failed\n",
                                    kPrioHALNotifierThread));
            }
        }

        if (!frc::SetProcessPriority("/usr/local/frc/bin/FRC_NetCommDaemon", true,
                                     kPrioNetCommDaemon)) {
            throw std::runtime_error(fmt::format(
                    "Giving /usr/local/frc/bin/FRC_NetCommDaemon RT priority {} failed",
                    kPrioNetCommDaemon));
        }

        StopCrond();
    }

    // These warnings generate console prints that cause scheduling jitter
    frc::DriverStation::SilenceJoystickConnectionWarning(true);

    // This telemetry regularly causes loop overruns
    frc::LiveWindow::DisableAllTelemetry();
#endif

    return frc::StartRobot<ToyotaViolinPlayingRobot>();
}

#endif