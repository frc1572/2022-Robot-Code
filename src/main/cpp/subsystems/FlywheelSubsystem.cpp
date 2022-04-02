#include "subsystems/FlywheelSubsystem.h"

#include <Eigen/Core>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>

#include "ctre/Phoenix.h"

FlywheelSubsystem::FlywheelSubsystem()
{
    m_leader.ConfigFactoryDefault();

    // Disable Talon FX velocity filtering so that our Kalman filter can do the
    // work

    // m_leader.ConfigVelocityMeasurementPeriod(SensorVelocityMeasPeriod::Period_1Ms);
    // m_leader.ConfigVelocityMeasurementWindow(1);
    m_leader.SetNeutralMode(Coast);

    m_leader.ConfigClosedLoopPeakOutput(0, .4);
    m_leader.Config_kP(0, .75);

    m_follower.ConfigFactoryDefault();
    m_follower.SetNeutralMode(Coast);
    m_follower.Follow(m_leader);
    m_follower.SetInverted(true);
    // Set Feeder to coast and config Default
}

void FlywheelSubsystem::Periodic()
{
    auto ffVoltage = units::volt_t{m_feedforward.Calculate(Eigen::Vector<double, 1>(m_desiredVelocity.value()))[0]};
    if (units::math::abs(ffVoltage) < Constants::MinimumFFVoltage)
    {
        ffVoltage = 0_V;
    }
    if (m_desiredVelocity > 0_deg_per_s)
    {
        m_leader.Set(
            ControlMode::Velocity,
            m_desiredVelocity * Constants::VelocityFactor::TalonFX * Constants::TicksPerRevolution::TalonFX,
            DemandType::DemandType_ArbitraryFeedForward,
            ffVoltage / 12_V);
    }
    else
    {
        m_leader.Set(ControlMode::PercentOutput, 0);
    }

    frc::SmartDashboard::PutNumber("Flywheel.MeasuredState", GetMeasuredVelocity().value());
    frc::SmartDashboard::PutNumber("Flywheel.DesiredState", GetDesiredVelocity().value());
}

void FlywheelSubsystem::SetSetpoint(rad_per_s_t setpoint)
{
    m_desiredVelocity = setpoint;
}

rad_per_s_t FlywheelSubsystem::GetDesiredVelocity()
{
    return m_desiredVelocity;
}

rad_per_s_t FlywheelSubsystem::GetMeasuredVelocity()
{
    return m_leader.GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX /
        Constants::VelocityFactor::TalonFX;
}