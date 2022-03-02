#include "subsystems/TurretSubsystem.h"

#include <iostream>

#include <frc/MathUtil.h>
#include <units/angle.h>

#include "Constants.h"

TurretSubsystem::TurretSubsystem()
{
    m_turret.ConfigFactoryDefault();
    m_turret.SetNeutralMode(Coast);
}
frc::Rotation2d TurretSubsystem::GetMeasuredPosition()
{
    return {
        (m_turret.GetSelectedSensorPosition() / Constants::TicksPerRevolution::TalonFX) /
        Constants::Turret::TurretGearing};
}
<<<<<<< Updated upstream
void TurretSubsystem::GetMotorVoltageSpike()
=======
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
}

void TurretSubsystem::Periodic()
{
    auto turretOffsetAngle = m_desiredPosition.Radians() + m_turretOffset / Constants::Turret::TurretGearing;

    turretFeedForward = m_turretFeedForward.Calculate(Eigen::Vector2d(turretOffsetAngle.value(), 0.0))[0];
    std::cout << "Turret Offset Position: " << m_turret.GetSelectedSensorPosition() << std::endl;

    m_turret.Set(
        ControlMode::Position,
        turretOffsetAngle * Constants::Turret::TurretGearing * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (turretFeedForward * 1_V + Constants::Turret::TurretKs * wpi::sgn(turretFeedForward)) / 12.0_V);
}

/*

void TurretSubsystem::GetCurrent()
>>>>>>> Stashed changes
{
    // TODO: Make method for returning bool based on if Spike in voltage
}
void TurretSubsystem::SetAbsolutePosition(double AbsolutePosition)
{
    while (m_turret.GetMotorOutputVoltage() > 1)
    {
        m_turret.Set(
            ControlMode::Velocity,
            1_rad_per_s * Constants::Turret::TurretGearing * Constants::VelocityFactor::TalonFX *
                Constants::TicksPerRevolution::TalonFX);
    }
    AbsolutePosition = m_turret.GetSelectedSensorPosition();
}

void TurretSubsystem::SetDesiredPosition(units::radian_t DesiredPosition)
{
    m_desiredPosition = DesiredPosition;
    units::radian_t TurretAngle = m_desiredPosition / Constants::Turret::TurretGearing;
    double TurretFeedForward = m_turretFeedForward.Calculate(Eigen::Vector2d(m_desiredPosition.value(), 0.0))[0];

    m_turret.Set(
        ControlMode::Position,
        TurretAngle * Constants::Turret::TurretGearing * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (TurretFeedForward * 1_V + Constants::Turret::TurretKs * wpi::sgn(TurretFeedForward)) / 12.0_V);
}