#include "subsystems/SwerveModuleSubsystem.h"
#include <memory>
#include <wpi/math>
#include <iostream>
#include "CustomUnits.h"
SwerveModuleSubsystem::SwerveModuleSubsystem(int throttlePort, int steeringPort, units::degree_t steeringOffset)
: m_throttleMotor(std::make_unique<WPI_TalonFX>(throttlePort)),
  m_steeringMotor(std::make_unique<WPI_TalonFX>(steeringPort)) {

    
    m_throttleMotor->ConfigFactoryDefault();
    m_throttleMotor->SetNeutralMode(NeutralMode::Coast);


    m_steeringMotor->ConfigFactoryDefault();
    m_steeringMotor->SetNeutralMode(NeutralMode::Coast);
    m_steeringMotor->ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    m_steeringMotor->ConfigIntegratedSensorOffset(steeringOffset.value());

    std::cout << "SwerveModuleSubsystem(" << throttlePort << ", " << steeringPort << ") " << m_steeringMotor->GetSelectedSensorPosition() << std::endl;

  }
     void SwerveModuleSubsystem::SetDesiredState(frc::SwerveModuleState desiredState){
         auto optimizedState = frc::SwerveModuleState::Optimize(desiredState, GetMeasuredRotation());
    rad_per_s_t SteeringVelocity = m_throttleMotor->GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX 
        / Constants::VelocityFactor::TalonFX;

    double throttlefeedforward = m_throttleFeedforward.Calculate(frc::MakeMatrix<1, 1>(GetMeasuredVelocity().value()), frc::MakeMatrix<1, 1>(optimizedState.speed.value())) [0];
    double steeringfeedforward = m_steeringFeedforward.Calculate(frc::MakeMatrix<1, 2> (GetMeasuredRotation().Radians().value(), SteeringVelocity.value()), frc::MakeMatrix<1, 2>(optimizedState.angle.Radians().value(), 0.0)) [0];

    m_throttleMotor->Set(
        ControlMode::Velocity,
        optimizedState.speed * Constants::TicksPerRevolution::TalonFX 
        * Constants::VelocityFactor::TalonFX / Constants::SwerveModule::WheelCircumference * 
        2_rad * wpi::numbers::pi * Constants::SwerveModule::Gearing, 
        DemandType::DemandType_ArbitraryFeedForward, throttlefeedforward / 12);
    m_steeringMotor->Set(
        ControlMode::Position,
        optimizedState.angle.Radians() * Constants::TicksPerRevolution::TalonFX,
        DemandType::DemandType_ArbitraryFeedForward,
        steeringfeedforward / 12);
}
     
    frc::Rotation2d SwerveModuleSubsystem::GetMeasuredRotation(){
        return{m_steeringMotor -> GetSelectedSensorPosition()/Constants::TicksPerRevolution::TalonFX};
    }
    frc::SwerveModuleState SwerveModuleSubsystem::GetMeasuredState(){
        return{.speed = GetMeasuredVelocity(), .angle=GetMeasuredRotation()};
    }

    decltype(0_mps) SwerveModuleSubsystem::GetMeasuredVelocity() {
        return
        m_throttleMotor->GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX 
        / Constants::VelocityFactor::TalonFX * Constants::SwerveModule::WheelCircumference / 
        2_rad / wpi::numbers::pi / Constants::SwerveModule::Gearing;
    };
