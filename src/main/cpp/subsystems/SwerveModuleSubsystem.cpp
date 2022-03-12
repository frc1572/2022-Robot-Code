#include "subsystems/SwerveModuleSubsystem.h"

#include <memory>

#include <Eigen/Core>
#include <fmt/format.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <spdlog/spdlog.h>
#include <wpi/numbers>

#include "CustomUnits.h"

SwerveModuleSubsystem::SwerveModuleSubsystem(int throttlePort, int steeringPort, units::radian_t steeringOffset)
  : m_throttleMotor(std::make_unique<WPI_TalonFX>(throttlePort, "canivore")),
    m_steeringMotor(std::make_unique<WPI_TalonFX>(steeringPort, "canivore")), m_steeringoffset(steeringOffset)
{
    SetName(fmt::format("SwerveModuleSubsystem({}, {})", throttlePort, steeringPort));

    m_throttleMotor->ConfigFactoryDefault();
    m_throttleMotor->SetNeutralMode(NeutralMode::Brake);
    if (frc::RobotBase::IsReal())
    {
        m_throttleMotor->SetInverted(false);
    }

    m_steeringMotor->ConfigFactoryDefault();
    m_steeringMotor->SetNeutralMode(NeutralMode::Brake);
    m_steeringMotor->ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    m_steeringMotor->ConfigClosedloopRamp(0.05);
    m_steeringMotor->Config_kP(0, 0.2);
    m_steeringMotor->Config_kD(0, 0.005);
    spdlog::info("{}, {}", throttlePort, m_steeringMotor->GetSelectedSensorPosition());
}

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
    bool shouldInvertThrottle = units::math::abs(frc::AngleModulus(stateDegrees - measuredDegrees)) > 90_deg;
    return {
        state.speed * (shouldInvertThrottle ? -1.0 : 1.0),
        nearest180 * 180_deg + stateDegrees,
    };
}

void SwerveModuleSubsystem::SetDesiredState(frc::SwerveModuleState desiredState)
{
    m_desiredState = {desiredState.speed, desiredState.speed == 0_mps ? m_desiredState.angle : desiredState.angle};
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
        Constants::VelocityFactor::TalonFX / Constants::SwerveModule::ThrottleGearing *
        Constants::SwerveModule::RolloutRatio;
}

void SwerveModuleSubsystem::TestingVoltage()
{
    m_throttleMotor->Set(ControlMode::PercentOutput, .1);
}

void SwerveModuleSubsystem::Periodic()
{
    auto optimizedState = OptimizeStateContinuous(m_desiredState);
    auto offsetAngle = optimizedState.angle.Radians() + m_steeringoffset / Constants::SwerveModule::SteeringGearing;

    auto throttlefeedforward =
        units::volt_t{m_throttleFeedforward.Calculate(Eigen::Vector<double, 1>(optimizedState.speed.value()))[0]};
    if (units::math::abs(throttlefeedforward) < Constants::MinimumFFVoltage)
    {
        throttlefeedforward = 0_V;
    }
    auto steeringfeedforward =
        units::volt_t{m_steeringFeedforward.Calculate(Eigen::Vector2d(offsetAngle.value(), 0.0))[0]};
    if (units::math::abs(steeringfeedforward) < Constants::MinimumFFVoltage)
    {
        steeringfeedforward = 0_V;
    }

    m_throttleMotor->Set(
        ControlMode::Velocity,
        optimizedState.speed / Constants::SwerveModule::RolloutRatio * Constants::SwerveModule::ThrottleGearing *
            Constants::VelocityFactor::TalonFX * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (throttlefeedforward + Constants::SwerveModule::ThrottleKs * wpi::sgn(throttlefeedforward)) / 12.0_V);
    m_steeringMotor->Set(
        ControlMode::Position,
        offsetAngle * Constants::SwerveModule::SteeringGearing * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        (steeringfeedforward + Constants::SwerveModule::SteeringKs * wpi::sgn(steeringfeedforward)) / 12.0_V);

    frc::SmartDashboard::PutNumber(
        fmt::format("{}.OutputVoltage", GetName()), m_throttleMotor->GetMotorOutputVoltage());

    frc::SmartDashboard::PutNumber(
        fmt::format("{}.RawThrottlePosition", GetName()), m_throttleMotor->GetSelectedSensorPosition());
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

void SwerveModuleSubsystem::SimulationPeriodic()
{
    m_throttleSim.SetInput(0, m_throttleMotor->GetMotorOutputVoltage());
    m_steeringSim.SetInput(0, m_steeringMotor->GetMotorOutputVoltage());

    m_throttleSim.Update(Constants::LoopPeriod);
    m_steeringSim.Update(Constants::LoopPeriod);

    auto throttleMotorSim = m_throttleMotor->GetSimCollection();
    auto throttleVelocityTicks = m_throttleSim.GetOutput(0) * 1_mps / Constants::SwerveModule::RolloutRatio *
        Constants::SwerveModule::ThrottleGearing * Constants::TicksPerRevolution::TalonFX;
    throttleMotorSim.SetIntegratedSensorVelocity(throttleVelocityTicks * Constants::VelocityFactor::TalonFX);
    throttleMotorSim.AddIntegratedSensorPosition(throttleVelocityTicks * Constants::LoopPeriod);

    auto steeringMotorSim = m_steeringMotor->GetSimCollection();
    auto steeringPositionTicks = m_steeringSim.GetOutput(0) * 1_rad * Constants::SwerveModule::SteeringGearing *
        Constants::TicksPerRevolution::TalonFX;
    steeringMotorSim.SetIntegratedSensorRawPosition(steeringPositionTicks);
}

void SwerveModuleSubsystem::Reset()
{
    m_desiredState = {};
    // TODO: use mag-encoders to reset integrated relative encoders
    m_throttleFeedforward.Reset(Eigen::Vector<double, 1>(GetMeasuredVelocity().value()));
    m_steeringFeedforward.Reset(Eigen::Vector<double, 2>(GetMeasuredRotation().Radians().value(), 0.0));
}