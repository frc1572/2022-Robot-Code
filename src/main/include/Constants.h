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
        constexpr Ks_t ThrottleKs = 0.5861_V;
        constexpr Kv_t<units::meter_t> ThrottleKv = 2.7241_V / 1_mps;
        constexpr Ka_t<units::meter_t> ThrottleKa = 0.084113_V / 1_mps_sq;

        constexpr Ks_t SteeringKs = 0.7027_V;
        constexpr Kv_t<units::radian_t> SteeringKv = 0.26826_V / 1_rad_per_s;
        constexpr Ka_t<units::radian_t> SteeringKa = 0.0008629_V / 1_rad_per_s_sq; // TODO: transcription error

        const double ThrottleGearing = 6.54;
        const double SteeringGearing = 72.0 / 14.0 * 24.0 / 8.0;
        constexpr auto WheelDiameter = 3.25_in;
        constexpr auto RolloutRatio = WheelDiameter / 2_rad;
        constexpr auto ThrottleMaxVelocity = 7_fps; // adjusted for driver, nominally 17 ft/s
        constexpr auto SteeringMaxVelocity = 1 * 360_deg_per_s;
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