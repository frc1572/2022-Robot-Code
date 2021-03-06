#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
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
    constexpr auto LoopPeriod = 1_s / 50;
    const frc::Translation2d GoalTranslation{27_ft, 13.5_ft}; // 27, 13.5
    constexpr auto UpperHubRadius = 26.69_in;
    constexpr auto CameraRotationRadius = 8.7333_in;
    constexpr auto MinimumFFVoltage = 1.0_V;

    namespace Systemspeeds
    {
        // double VairableHoodSpeed;
        constexpr double TurretFeederSpeed = .5;
        constexpr double IntakeFeederSpeed = 0.5;
        constexpr double IntakeSpeed = 0.45;
        constexpr double HoodSpeed = 1900;
        constexpr double HoodReverseSpeed = 6000;
        constexpr double WinchOutput = 1.0;
        constexpr double WinchRelease = 0.25;

    } // namespace Systemspeeds
    namespace Flywheel
    {
        constexpr int LeaderID = 12;
        constexpr int FollowerID = 15;
        constexpr int FeederID = 11;
        constexpr Ks_t Ks = 1_V;
        constexpr Kv_t<units::radian_t> Kv = 0.025_V / 1_rad_per_s;
        constexpr Ka_t<units::radian_t> Ka = 0.0025_V / 1_rad_per_s_sq;
        constexpr auto Diameter = 4_in;
        constexpr auto Angle = 23.68_deg;
        constexpr double FlyWheelGearing = 1.5;
    } // namespace Flywheel

    namespace SwerveModule
    {
        constexpr Ks_t ThrottleKs = 0.5861_V;
        constexpr Kv_t<units::meter_t> ThrottleKv = 2.7241_V / 1_mps;
        constexpr Ka_t<units::meter_t> ThrottleKa = 0.084113_V / 1_mps_sq;

        constexpr Ks_t SteeringKs = 0.7027_V;
        constexpr Kv_t<units::radian_t> SteeringKv = 0.26826_V / 1_rad_per_s;
        constexpr Ka_t<units::radian_t> SteeringKa =
            0.0008629_V / 1_rad_per_s_sq; // TODO:\\\\\\\\\\\\\\\\0 transcription error
        const double ThrottleGearing = 6.54;
        const double SteeringGearing = 72.0 / 14.0 * 24.0 / 8.0;
        constexpr auto WheelDiameter = 3.25_in;
        constexpr auto RolloutRatio = WheelDiameter / 2_rad;
        constexpr auto ThrottleMaxVelocity = 17_fps; // adjusted for driver, nominally 17 ft/s
        constexpr auto SteeringMaxVelocity = 1 * 360_deg_per_s;
    } // namespace SwerveModule

    namespace Turret
    {
        constexpr int TurretPort = 9;
        constexpr double TurretGearing = 70.0;
        // double Turretposition;
        constexpr Ks_t TurretKs = 0.67361_V;
        constexpr Kv_t<units::radian_t> TurretKv = 1.2113_V / 1_rad_per_s;
        constexpr Ka_t<units::radian_t> TurretKa = 0.030709_V / 1_rad_per_s_sq;
        constexpr auto TurretMaxVelocity = 1 * 360_deg_per_s;
    } // namespace Turret

    namespace TicksPerRevolution
    {
        constexpr auto TalonFX = 2048 / 2_rad / wpi::numbers::pi;
    } // namespace TicksPerRevolution

    namespace VelocityFactor
    {
        constexpr auto TalonFX = 100_ms;
    } // namespace VelocityFactor
    namespace IntakeSystem
    {
        constexpr int IntakeID = 14; // 14
        constexpr int MainFeederID = 13;
    } // namespace IntakeSystem
    namespace Climb
    {
        constexpr int WinchID = 16;
    } // namespace Climb
} // namespace Constants