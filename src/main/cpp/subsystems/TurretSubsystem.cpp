#include "subsystems/TurretSubsystem.h"

#include <iostream>

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>

#include "Constants.h"
#include "ctre/Phoenix.h"
#include "helper/spdlog.h"

TurretSubsystem::TurretSubsystem()
{
    m_turret.ConfigFactoryDefault();
    m_turret.SetNeutralMode(Coast);
    m_turret.SetInverted(true);
    m_turret.Config_kP(0, 0.05);
    m_turret.Config_kD(0, 0.005);

    SetName(fmt::format("TurretSubsystem({})", m_turret.GetDeviceID()));
}
/*
frc::Rotation2d TurretSubsystem::GetMeasuredPosition()
{
    return {
        (m_turret.GetSelectedSensorPosition() / Constants::TicksPerRevolution::TalonFX) *
        Constants::Turret::TurretGearing};
}
*/

frc::Rotation2d TurretSubsystem::GetMeasuredPosition()
{
    return {
        m_turret.GetSelectedSensorPosition() / Constants::TicksPerRevolution::TalonFX /
        Constants::Turret::TurretGearing};
}

void TurretSubsystem::SetDesiredPosition(frc::Rotation2d desiredPosition)
{
    m_desiredPosition = desiredPosition;
    std::cout << "SetDesiredPosition: Running" << std::endl;
}

decltype(1_rad_per_s) TurretSubsystem::GetMeasuredVelocity()
{
    return m_turret.GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX /
        Constants::VelocityFactor::TalonFX / Constants::Turret::TurretGearing;
}

void TurretSubsystem::Periodic()
{
    auto turretOffsetAngle = m_desiredPosition.Radians() + m_turretOffset / Constants::Turret::TurretGearing;
    double turretFeedForward = m_turretFeedForward.Calculate(Eigen::Vector2d(turretOffsetAngle.value(), 0.0))[0];
    // std::cout << "Turret Offset Number: " << m_turret.GetSelectedSensorPosition() << std::endl;

    m_turret.Set(
        ControlMode::Position,
        turretOffsetAngle * Constants::Turret::TurretGearing * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (turretFeedForward * 1_V + Constants::Turret::TurretKs * wpi::sgn(turretFeedForward)) / 12.0_V);

    frc::SmartDashboard::PutNumber(fmt::format("{}.RawPosition", GetName()), m_turret.GetSelectedSensorPosition());
    // std::cout << "Periodic: Running" << std::endl;
    // std::cout << m_desiredPosition.Degrees().value() << std::endl;
}

/*

void TurretSubsystem::GetCurrent()
{
    return m_turret.GetStatorCurrent();
}


void TurretSubsystem::AddDesiredPosition(units::radian_t positionOffset)
{
    m_desiredPosition += positionOffset;
}

void TurretSubsystem::Periodic()
{

    double ffOutput = m_turretFeedForward.Calculate(Eigen::Vector2d(m_desiredPosition.value(), 0.0))[0];

    m_turret.Set(
        ControlMode::Position,
        m_desiredPosition * Constants::Turret::TurretGearing * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (ffOutput * 1_V + Constants::Turret::TurretKs * wpi::sgn(ffOutput)) / 12.0_V);

}
*/