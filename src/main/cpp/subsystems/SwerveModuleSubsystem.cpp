#include "subsystems/SwerveModuleSubsystem.h"

#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <fmt/format.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/numbers>

#include "CustomUnits.h"

SwerveModuleSubsystem::SwerveModuleSubsystem(int throttlePort, int steeringPort, units::radian_t steeringOffset)
  : m_throttleMotor(std::make_unique<WPI_TalonFX>(throttlePort)),
    m_steeringMotor(std::make_unique<WPI_TalonFX>(steeringPort)), m_steeringoffset(steeringOffset)
{
    SetName(fmt::format("SwerveModuleSubsystem({}, {})", throttlePort, steeringPort));

    m_throttleMotor->ConfigFactoryDefault();
    m_throttleMotor->SetNeutralMode(NeutralMode::Coast);

    m_steeringMotor->ConfigFactoryDefault();
    m_steeringMotor->SetNeutralMode(NeutralMode::Brake);
    m_steeringMotor->ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
}

void SwerveModuleSubsystem::SetDesiredState(frc::SwerveModuleState desiredState)
{
    auto optimizedState = frc::SwerveModuleState::Optimize(desiredState, GetMeasuredRotation());
    auto offsetAngle = optimizedState.angle.Radians() + m_steeringoffset;

    double throttlefeedforward =
        m_throttleFeedforward.Calculate(Eigen::Vector<double, 1>(optimizedState.speed.value()))[0];
    double steeringfeedforward = m_steeringFeedforward.Calculate(Eigen::Vector2d(offsetAngle.value(), 0.0))[0];

    m_throttleMotor->Set(
        ControlMode::Velocity,
        optimizedState.speed / Constants::SwerveModule::RolloutRatio * Constants::SwerveModule::ThrottleGearing *
            Constants::VelocityFactor::TalonFX * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (throttlefeedforward + Constants::SwerveModule::ThrottleKs.value() * wpi::sgn(throttlefeedforward)) / 12.0);
    m_steeringMotor->Set(
        ControlMode::Position,
        offsetAngle * Constants::SwerveModule::SteeringGearing * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (steeringfeedforward + Constants::SwerveModule::SteeringKs.value() * wpi::sgn(steeringfeedforward)) / 12.0);

    frc::SmartDashboard::PutNumber(fmt::format("{}.DesiredThrottleVelocity", GetName()), optimizedState.speed.value());
    frc::SmartDashboard::PutNumber(
        fmt::format("{}.DesiredSteeringPosition", GetName()), optimizedState.angle.Radians().value());
}

frc::Rotation2d SwerveModuleSubsystem::GetMeasuredRotation()
{
    return {
        (m_steeringMotor->GetSelectedSensorPosition() / Constants::TicksPerRevolution::TalonFX - m_steeringoffset) /
        Constants::SwerveModule::SteeringGearing};
}
frc::SwerveModuleState SwerveModuleSubsystem::GetMeasuredState()
{
    return {GetMeasuredVelocity(), GetMeasuredRotation()};
}

decltype(0_mps) SwerveModuleSubsystem::GetMeasuredVelocity()
{
    return m_throttleMotor->GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX /
        Constants::VelocityFactor::TalonFX * Constants::SwerveModule::ThrottleGearing *
        Constants::SwerveModule::RolloutRatio;
}

void SwerveModuleSubsystem::TestingVoltage()
{
    m_throttleMotor->Set(ControlMode::PercentOutput, .1);
}

void SwerveModuleSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber(
        fmt::format("{}.MeasuredThrottleVelocity", GetName()), GetMeasuredVelocity().value());
    frc::SmartDashboard::PutNumber(
        fmt::format("{}.MeasuredSteeringPosition", GetName()), GetMeasuredRotation().Radians().value());
}