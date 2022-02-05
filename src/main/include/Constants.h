#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/numbers>

#include "CustomUnits.h"

namespace Constants
{
    constexpr auto LoopPeriod = 1_s / 100;

    namespace Flywheel
    {
        constexpr int LeaderID = 0;
        constexpr Ks_t Ks = 1_V;
        constexpr Kv_t<units::radian_t> Kv = 0.025_V / 1_rad_per_s;
        constexpr Ka_t<units::radian_t> Ka = 0.0025_V / 1_rad_per_s_sq;
    } // namespace Flywheel

    namespace SwerveModule
    {
        constexpr Ks_t throttleKs = 1_V;
        constexpr Kv_t<units::meter_t> throttleKv = 0.025_V / 1_mps;
        constexpr Ka_t<units::meter_t> throttleKa = 0.0025_V / 1_mps_sq;

        constexpr Ks_t steeringKs = 1_V;
        constexpr Kv_t<units::radian_t> steeringKv = 0.025_V / 1_rad_per_s;
        constexpr Ka_t<units::radian_t> steeringKa = 0.0025_V / 1_rad_per_s_sq;

        const double Gearing = 6.54;
        constexpr auto WheelDiameter = 3.25_in;
        constexpr auto WheelCircumference = WheelDiameter * wpi::numbers::pi;
    } // namespace SwerveModule

    namespace TicksPerRevolution
    {
        constexpr auto TalonFX = 2048 / 2_rad / wpi::numbers::pi;
    } // namespace TicksPerRevolution

    namespace VelocityFactor
    {
        constexpr auto TalonFX = 100_ms;
    } // namespace VelocityFactor
} // namespace Constants