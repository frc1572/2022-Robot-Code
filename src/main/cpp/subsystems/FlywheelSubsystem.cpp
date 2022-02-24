#include "subsystems/FlywheelSubsystem.h"

#include <Eigen/Core>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>

#include "ctre/Phoenix.h"

FlywheelSubsystem::FlywheelSubsystem()
{
    m_leader.ConfigFactoryDefault();
    m_feeder.ConfigFactoryDefault();
    // Disable Talon FX velocity filtering so that our Kalman filter can do the
    // work
    m_leader.ConfigVelocityMeasurementPeriod(SensorVelocityMeasPeriod::Period_1Ms);
    m_leader.ConfigVelocityMeasurementWindow(1);

    m_feeder.SetNeutralMode(Coast);
    // Set Feeder to coast and config Default
    frc::SmartDashboard::PutNumber("Flywheel.Setpoint", 0);
}

void FlywheelSubsystem::Periodic()
{
    rad_per_s_t velocity = m_leader.GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX /
        Constants::VelocityFactor::TalonFX;
    m_loop.Correct(Eigen::Vector<double, 1>(velocity.to<double>()));
    m_loop.Predict(Constants::LoopPeriod);
    auto voltage = m_loop.U(0);
    m_leader.SetVoltage(voltage * 1_V + wpi::sgn(voltage) * Constants::Flywheel::Ks);

    if (m_loop.NextR(0) == 0)
    {
        m_leader.Set(ControlMode::PercentOutput, 0.0);
        // m_leader.SetVoltage((voltage * 1_V + wpi::sgn(voltage) * Constants::Flywheel::Ks) * 0);
    }

    frc::SmartDashboard::PutNumber("Flywheel.MeasuredState", velocity.to<double>());
    frc::SmartDashboard::PutNumber("Flywheel.EstimatedState", m_loop.Xhat(0));
}

void FlywheelSubsystem::SimulationPeriodic()
{
    auto motorVoltage = m_leader.GetMotorOutputVoltage() * 1_V;
    auto systemVoltage = motorVoltage - wpi::sgn(motorVoltage) * Constants::Flywheel::Ks;
    if (systemVoltage > -Constants::Flywheel::Ks && systemVoltage < Constants::Flywheel::Ks)
    {
        systemVoltage = 0_V;
    }
    m_plantSim.SetInput(0, systemVoltage / 1_V);
    m_plantSim.Update(Constants::LoopPeriod);
    auto output = m_plantSim.GetOutput(0) * 1_rad_per_s * Constants::TicksPerRevolution::TalonFX;
    auto leaderSim = m_leader.GetSimCollection();
    leaderSim.SetIntegratedSensorVelocity(output * Constants::VelocityFactor::TalonFX);
    leaderSim.AddIntegratedSensorPosition(output * Constants::LoopPeriod);
}

void FlywheelSubsystem::SetSetpoint(rad_per_s_t setpoint)
{
    m_loop.SetNextR(Eigen::Vector<double, 1>(setpoint.to<double>()));

    frc::SmartDashboard::PutNumber("Flywheel.Setpoint", setpoint.to<double>());
}

void FlywheelSubsystem::StartFeeder(double FeedRpm)
{
    m_feeder.Set(ControlMode::PercentOutput, FeedRpm);
}
// Code for the StartFeeder^^   Could maybe add it too periodic eventually