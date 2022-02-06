#include "subsystems/SwerveModuleSubsystem.h"

#include <iostream>
#include <memory>

#include <wpi/math>

#include "CustomUnits.h"

SwerveModuleSubsystem::SwerveModuleSubsystem(int throttlePort, int steeringPort, units::radian_t steeringOffset)
  : m_throttleMotor(std::make_unique<WPI_TalonFX>(throttlePort)),
    m_steeringMotor(std::make_unique<WPI_TalonFX>(steeringPort)), m_steeringoffset(steeringOffset)
{
    m_throttleMotor->ConfigFactoryDefault();
    m_throttleMotor->SetNeutralMode(NeutralMode::Coast);

    m_steeringMotor->ConfigFactoryDefault();
    m_steeringMotor->SetNeutralMode(NeutralMode::Coast);
    m_steeringMotor->ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);

    std::cout << "SwerveModuleSubsystem(" << throttlePort << ", " << steeringPort << ") "
              << GetMeasuredRotation().Degrees().value() << std::endl;
}

void SwerveModuleSubsystem::SetDesiredState(frc::SwerveModuleState desiredState)
{
    auto optimizedState = frc::SwerveModuleState::Optimize(desiredState, GetMeasuredRotation());
    auto OffsetAngle = optimizedState.angle.Radians() + m_steeringoffset;
    rad_per_s_t SteeringVelocity = m_steeringMotor->GetSelectedSensorVelocity() /
        Constants::TicksPerRevolution::TalonFX / Constants::VelocityFactor::TalonFX /
        Constants::SwerveModule::SteeringGearing * -1.0;

    double throttlefeedforward = m_throttleFeedforward.Calculate(
        frc::MakeMatrix<1, 1>(GetMeasuredVelocity().value()), frc::MakeMatrix<1, 1>(optimizedState.speed.value()))[0];
    double steeringfeedforward = m_steeringFeedforward.Calculate(
        frc::MakeMatrix<1, 2>(GetMeasuredRotation().Radians().value(), SteeringVelocity.value()),
        frc::MakeMatrix<1, 2>(OffsetAngle.value(), 0.0))[0];

    m_throttleMotor->Set(
        ControlMode::Velocity,
        optimizedState.speed * Constants::TicksPerRevolution::TalonFX * Constants::VelocityFactor::TalonFX /
            Constants::SwerveModule::WheelCircumference * 2_rad * wpi::numbers::pi * Constants::SwerveModule::Gearing,
        DemandType::DemandType_ArbitraryFeedForward,
        throttlefeedforward / 12);
    m_steeringMotor->Set(
        ControlMode::Position,
        OffsetAngle * Constants::TicksPerRevolution::TalonFX * Constants::SwerveModule::SteeringGearing * -1.0,
        DemandType::DemandType_ArbitraryFeedForward,
        steeringfeedforward / 12 * -1.0);
}

frc::Rotation2d SwerveModuleSubsystem::GetMeasuredRotation()
{
    return {
        (m_steeringMotor->GetSelectedSensorPosition() / Constants::TicksPerRevolution::TalonFX - m_steeringoffset) /
        Constants::SwerveModule::SteeringGearing * -1.0};
}
frc::SwerveModuleState SwerveModuleSubsystem::GetMeasuredState()
{
    return {.speed = GetMeasuredVelocity(), .angle = GetMeasuredRotation()};
}

decltype(0_mps) SwerveModuleSubsystem::GetMeasuredVelocity()
{
    return m_throttleMotor->GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX /
        Constants::VelocityFactor::TalonFX * Constants::SwerveModule::WheelCircumference / 2_rad / wpi::numbers::pi /
        Constants::SwerveModule::Gearing;
}

void SwerveModuleSubsystem::TestingVoltage()
{
    m_throttleMotor->Set(ControlMode::PercentOutput, .1);
};
