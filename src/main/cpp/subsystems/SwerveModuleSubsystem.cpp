#include "subsystems/SwerveModuleSubsystem.h"

#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <fmt/format.h>
#include <frc/MathUtil.h>
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
    // m_steeringMotor->SetSensorPhase(true);
    // m_steeringMotor->ConfigClosedloopRamp(0.25);
    m_steeringMotor->Config_kP(0, 0.5);
    m_steeringMotor->Config_kD(0, 0.1);
    std::cout << throttlePort << ", " << m_steeringMotor->GetSelectedSensorPosition() << std::endl;
}

void SwerveModuleSubsystem::SetDesiredState(frc::SwerveModuleState desiredState)
{
    if (desiredState.speed == 0_mps)
    {
        desiredState.angle = m_desiredState.angle;
    }
    m_desiredState = desiredState;

    // Optimizes desiredState for our continuous-rotation modules
    frc::SwerveModuleState optimizedState;
    int halfTurns = std::round(((GetMeasuredRotation().Degrees() - desiredState.angle.Degrees()) / 180_deg).value());
    optimizedState.angle = halfTurns * 180_deg + desiredState.angle.Degrees();
    optimizedState.speed = desiredState.speed;
    if (units::math::abs(desiredState.angle.Degrees() - frc::AngleModulus(GetMeasuredRotation().Degrees())) > 90_deg)
    {
        optimizedState.speed *= -1;
    }
    auto offsetAngle = optimizedState.angle.Radians() + m_steeringoffset / Constants::SwerveModule::SteeringGearing;

    double throttlefeedforward =
        m_throttleFeedforward.Calculate(Eigen::Vector<double, 1>(optimizedState.speed.value()))[0];
    double steeringfeedforward = m_steeringFeedforward.Calculate(Eigen::Vector2d(offsetAngle.value(), 0.0))[0];

    m_throttleMotor->Set(
        ControlMode::Velocity,
        optimizedState.speed / Constants::SwerveModule::RolloutRatio * Constants::SwerveModule::ThrottleGearing *
            Constants::VelocityFactor::TalonFX * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (throttlefeedforward * 1_V + Constants::SwerveModule::ThrottleKs * wpi::sgn(throttlefeedforward)) / 12.0_V);
    m_steeringMotor->Set(
        ControlMode::Position,
        offsetAngle * Constants::SwerveModule::SteeringGearing * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (steeringfeedforward * 1_V + Constants::SwerveModule::SteeringKs * wpi::sgn(steeringfeedforward)) / 12.0_V);

    frc::SmartDashboard::PutNumber(
        fmt::format("{}.RawPosition", GetName()), m_steeringMotor->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber(fmt::format("{}.DesiredThrottleVelocity", GetName()), optimizedState.speed.value());
    frc::SmartDashboard::PutNumber(
        fmt::format("{}.DesiredSteeringPosition", GetName()), optimizedState.angle.Radians().value());
    frc::SmartDashboard::PutNumber(
        fmt::format("{}.MeasuredThrottleVelocity", GetName()), GetMeasuredVelocity().value());
    frc::SmartDashboard::PutNumber(
        fmt::format("{}.MeasuredSteeringPosition", GetName()), GetMeasuredRotation().Radians().value());
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

// void SwerveModuleSubsystem::Periodic()
// {
//     std::cout << "periodic" << std::endl;
// }