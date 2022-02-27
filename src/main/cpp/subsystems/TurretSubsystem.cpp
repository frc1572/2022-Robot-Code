#include "subsystems/TurretSubsystem.h"

#include <frc/MathUtil.h>
#include <units/angle.h>

#include "Constants.h"
#include "ctre/Phoenix.h"

TurretSubsystem::TurretSubsystem()
{
    m_turret.ConfigFactoryDefault();
    m_turret.SetNeutralMode(Coast);
}
/*
frc::Rotation2d TurretSubsystem::GetMeasuredPosition()
{
    return {
        (m_turret.GetSelectedSensorPosition() / Constants::TicksPerRevolution::TalonFX) *
        Constants::Turret::TurretGearing};
}
*/

void TurretSubsystem::SetDesiredPosition(double desiredPosition)
{
    m_turret.Set(ControlMode::PercentOutput, desiredPosition);
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