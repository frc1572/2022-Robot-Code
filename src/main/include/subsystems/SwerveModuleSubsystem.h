#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <ctre/Phoenix.h>
#include <units/angle.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include "Constants.h"

class SwerveModuleSubsystem : public frc2::SubsystemBase {
public:
SwerveModuleSubsystem(int throttlePort, int steeringPort, units::degree_t steeringOffset);
  void SetDesiredState(frc::SwerveModuleState desiredState);
    frc::Rotation2d GetMeasuredRotation();
    frc::SwerveModuleState GetMeasuredState();
    decltype(0_mps) GetMeasuredVelocity();

private: 
    std::unique_ptr<WPI_TalonFX> m_throttleMotor;
    std::unique_ptr<WPI_TalonFX> m_steeringMotor;

    const frc::LinearSystem<1, 1, 1> m_throttleSystem = frc::LinearSystemId::IdentifyVelocitySystem<units::meter>(
        Constants::SwerveModule::throttleKv,
        Constants::SwerveModule::throttleKa);
    const frc::LinearSystem<2, 1, 1> m_steeringSystem = frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
        Constants::SwerveModule::steeringKv,
        Constants::SwerveModule::steeringKa);
    frc::LinearPlantInversionFeedforward<1,1> m_throttleFeedforward{ m_throttleSystem, Constants::LoopPeriod};
    frc::LinearPlantInversionFeedforward<2,1> m_steeringFeedforward{ m_steeringSystem, Constants::LoopPeriod};
};

