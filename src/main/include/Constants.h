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
    constexpr auto LoopPeriod = 1_s / 100;
    const frc::Translation2d GoalTranslation{15_ft, 0_ft};
    constexpr auto UpperHubRadius = 26.69_in;
    // frc::Translation2d GoalTranslation{54_ft / 2, 27_ft / 2};
    constexpr auto CameraRotationRadius = 8.7333_in;
    constexpr auto MinimumFFVoltage = 0.1_V;

    namespace Systemspeeds
    {
        constexpr double TurretFeederSpeed = 0.7;
        constexpr double IntakeFeederSpeed = 0.5;
        constexpr double IntakeSpeed = 0.2;
        constexpr double HoodSpeed = 4500;
    } // namespace Systemspeeds
    namespace Flywheel
    {
        constexpr int LeaderID = 14;
        constexpr int FeederID = 11;
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
        constexpr auto ThrottleMaxVelocity = 10_fps; // adjusted for driver, nominally 17 ft/s
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
        constexpr int IntakeID = 12; // 14
        constexpr int MainFeederID = 13;
    } // namespace IntakeSystem
} // namespace Constants