#include "subsystems/TurretSubsystem.h"

#include <iostream>

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <spdlog/spdlog.h>
#include <units/angle.h>

#include "Constants.h"
#include "ctre/Phoenix.h"
#include "frc/RobotBase.h"

TurretSubsystem::TurretSubsystem()
{
    m_turret.ConfigFactoryDefault();
    m_turret.SetNeutralMode(NeutralMode::Brake);
    if (frc::RobotBase::IsReal())
    {
        m_turret.SetInverted(true);
    }
    m_turret.Config_kP(0, 0.225);
    m_turret.Config_kD(0, 0.15);
    // m_turret.ConfigClosedLoopPeakOutput(0, .75);
    SetName(fmt::format("TurretSubsystem({})", m_turret.GetDeviceID()));
}

frc::Rotation2d TurretSubsystem::GetMeasuredRotation()
{
    return frc::Rotation2d{
               m_turret.GetSelectedSensorPosition() / Constants::TicksPerRevolution::TalonFX /
               Constants::Turret::TurretGearing} +
        m_rotationOffset;
}

void TurretSubsystem::SetDesiredPosition(frc::Rotation2d desiredPosition, rad_per_s_t desiredVelocity)
{
    m_desiredPosition = desiredPosition;
    m_desiredVelocity = desiredVelocity;
}

decltype(1_rad_per_s) TurretSubsystem::GetMeasuredVelocity()
{
    return m_turret.GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX /
        Constants::VelocityFactor::TalonFX / Constants::Turret::TurretGearing;
}

void TurretSubsystem::Periodic()
{
    auto turretOffsetAngle = frc::AngleModulus(m_desiredPosition.Radians()) - m_rotationOffset.Radians();
    auto turretFeedForward = units::volt_t{
        m_turretFeedForward.Calculate(Eigen::Vector2d(turretOffsetAngle.value(), m_desiredVelocity.value()))[0]};
    if (units::math::abs(turretFeedForward) < Constants::MinimumFFVoltage)
    {
        turretFeedForward = 0_V;
    }

    m_turret.Set(
        ControlMode::Position,
        turretOffsetAngle * Constants::Turret::TurretGearing * Constants::TicksPerRevolution::TalonFX /*,
        DemandType::DemandType_ArbitraryFeedForward,
        (turretFeedForward + Constants::Turret::TurretKs * wpi::sgn(turretFeedForward)) / 12.0_V*/);

    frc::SmartDashboard::PutNumber(
        fmt::format("{}.DesiredRotationDegrees", GetName()), m_desiredPosition.Degrees().value());
    frc::SmartDashboard::PutNumber(
        fmt::format("{}.MeasuredRotationDegrees", GetName()), GetMeasuredRotation().Degrees().value());
}

void TurretSubsystem::SimulationPeriodic()
{
    m_turretSim.SetInput(0, m_turret.GetMotorOutputVoltage());
    m_turretSim.Update(Constants::LoopPeriod);

    auto turretMotorSim = m_turret.GetSimCollection();
    auto turretPositionTicks =
        m_turretSim.GetOutput(0) * 1_rad * Constants::Turret::TurretGearing * Constants::TicksPerRevolution::TalonFX;
    turretMotorSim.SetIntegratedSensorRawPosition(turretPositionTicks);
}

void TurretSubsystem::Reset(frc::Rotation2d currentRotation)
{
    m_rotationOffset = 0_deg;
    m_rotationOffset = currentRotation - GetMeasuredRotation();
}