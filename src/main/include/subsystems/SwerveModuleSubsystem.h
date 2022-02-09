#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

#include "Constants.h"

class SwerveModuleSubsystem : public frc2::SubsystemBase
{
public:
    SwerveModuleSubsystem(
        int throttlePort, int steeringPort, int absoluteEncoderPort, units::degree_t absoluteEncoderOffset);
    frc::SwerveModuleState OptimizeStateContinuous(frc::SwerveModuleState state);
    void SetDesiredState(frc::SwerveModuleState desiredState);
    frc::Rotation2d GetMeasuredRotation();
    frc::SwerveModuleState GetMeasuredState();
    decltype(0_mps) GetMeasuredVelocity();
    void TestingVoltage();
    void Periodic(int absoluteEncoderPort) override;
    // void Periodic() override;

private:
    std::unique_ptr<WPI_TalonFX> m_throttleMotor;
    std::unique_ptr<WPI_TalonFX> m_steeringMotor;
    frc::DutyCycleEncoder m_absoluteEncoder;

    frc::SwerveModuleState m_desiredState;

    const frc::LinearSystem<1, 1, 1> m_throttleSystem = frc::LinearSystemId::IdentifyVelocitySystem<units::meter>(
        Constants::SwerveModule::ThrottleKv, Constants::SwerveModule::ThrottleKa);
    const frc::LinearSystem<2, 1, 1> m_steeringSystem = frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
        Constants::SwerveModule::SteeringKv, Constants::SwerveModule::SteeringKa);
    frc::LinearPlantInversionFeedforward<1, 1> m_throttleFeedforward{m_throttleSystem, Constants::LoopPeriod};
    frc::LinearPlantInversionFeedforward<2, 1> m_steeringFeedforward{m_steeringSystem, Constants::LoopPeriod};
};
