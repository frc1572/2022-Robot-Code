#include "subsystems/SwerveModuleSubsystem.h"

#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <fmt/format.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/numbers>

#include "CustomUnits.h"

SwerveModuleSubsystem::SwerveModuleSubsystem(
    int throttlePort, int steeringPort, int absoluteEncoderPort, units::degree_t absoluteEncoderOffset)
  : m_throttleMotor(std::make_unique<WPI_TalonFX>(throttlePort, "rio")),
    m_steeringMotor(std::make_unique<WPI_TalonFX>(steeringPort, "rio")), m_absoluteEncoder(absoluteEncoderPort)
{
    SetName(fmt::format("SwerveModuleSubsystem({}, {})", throttlePort, steeringPort));

    m_throttleMotor->ConfigFactoryDefault();
    m_throttleMotor->SetNeutralMode(NeutralMode::Brake);

    m_steeringMotor->ConfigFactoryDefault();
    m_steeringMotor->SetNeutralMode(NeutralMode::Brake);
    m_steeringMotor->SetSelectedSensorPosition(
        (m_absoluteEncoder.Get() + absoluteEncoderOffset) * Constants::SwerveModule::SteeringGearing *
        Constants::TicksPerRevolution::TalonFX);
    m_steeringMotor->ConfigClosedloopRamp(0.05);
    m_steeringMotor->Config_kP(0, 0.2);
    m_steeringMotor->Config_kD(0, 0.005);
}

// Temp Testing of the Encoders VVV (>>>Might be broken<<< I may have done this wrong, sorry lol)
/*
void SwerveModuleSubsystem::Periodic()
{
    std::cout << m_absoluteEncoder.GetSourceChannel() << " - " << m_absoluteEncoder.Get().value() << std::endl;
}
*/
frc::SwerveModuleState SwerveModuleSubsystem::OptimizeStateContinuous(frc::SwerveModuleState state)
{
    // Optimizes module state for a continuously rotating module.
    //
    // For example, if the desired angle is 10 degrees, then 10, 190, 370, etc. are all valid angles as well. The same
    // goes for the negative direction. Given our current angle, we want to find the nearest member of that sequence
    // and use it as our optimized angle. We do this by factoring the desired angle out of both the sequence and our
    // current angle, then finding x s.t. x * 180 is closest to the adjusted current angle.
    auto measuredDegrees = GetMeasuredRotation().Degrees();
    auto stateDegrees = state.angle.Degrees();
    int nearest180 = units::math::round((measuredDegrees - stateDegrees) / 180_deg);
    return {
        state.speed * ((units::math::abs(stateDegrees - frc::AngleModulus(measuredDegrees)) > 90_deg) ? -1.0 : 1.0),
        nearest180 * 180_deg + stateDegrees,
    };
}

void SwerveModuleSubsystem::SetDesiredState(frc::SwerveModuleState desiredState)
{
    if (desiredState.speed == 0_mps)
    {
        desiredState.angle = m_desiredState.angle;
    }
    m_desiredState = desiredState;

    auto optimizedState = OptimizeStateContinuous(desiredState);
    auto motorAngle = optimizedState.angle.Radians();

    double throttlefeedforward =
        m_throttleFeedforward.Calculate(Eigen::Vector<double, 1>(optimizedState.speed.value()))[0];
    double steeringfeedforward = m_steeringFeedforward.Calculate(Eigen::Vector2d(motorAngle.value(), 0.0))[0];

    m_throttleMotor->Set(
        ControlMode::Velocity,
        optimizedState.speed / Constants::SwerveModule::RolloutRatio * Constants::SwerveModule::ThrottleGearing *
            Constants::VelocityFactor::TalonFX * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (throttlefeedforward * 1_V + Constants::SwerveModule::ThrottleKs * wpi::sgn(throttlefeedforward)) / 12.0_V);
    m_steeringMotor->Set(
        ControlMode::Position,
        motorAngle * Constants::SwerveModule::SteeringGearing * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (steeringfeedforward * 1_V + Constants::SwerveModule::SteeringKs * wpi::sgn(steeringfeedforward)) / 12.0_V);

    std::cout << m_absoluteEncoder.GetSourceChannel() << " - " << m_absoluteEncoder.Get().value() << std::endl;
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
        units::radian_t(m_steeringMotor->GetSelectedSensorPosition() / Constants::TicksPerRevolution::TalonFX) /
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