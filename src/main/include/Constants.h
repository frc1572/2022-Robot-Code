#pragma once

#include <units/angular_acceleration.h>
#include <wpi/numbers>

#include "CustomUnits.h"

namespace Constants {
    constexpr auto LoopPeriod = 1_s / 100;

    namespace Flywheel {
        constexpr int LeaderID = 1;
        constexpr Ks_t Ks = 0.93373_V;
        constexpr Kv_t<units::radian_t> Kv = 0.018794_V / 1_rad_per_s;
        constexpr Ka_t<units::radian_t> Ka = 0.0014677_V / 1_rad_per_s_sq;
    }

    namespace TicksPerRevolution {
        constexpr auto TalonFX = 2048 / 2_rad / wpi::numbers::pi;
    }

    namespace VelocityFactor {
        constexpr auto TalonFX = 100_ms;
    }
}