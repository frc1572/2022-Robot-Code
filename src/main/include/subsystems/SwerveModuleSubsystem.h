#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
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
    SwerveModuleSubsystem(int throttlePort, int steeringPort, units::radian_t steeringOffset);
    void SetDesiredState(frc::SwerveModuleState desiredState);
    frc::Rotation2d GetMeasuredRotation();
    frc::SwerveModuleState GetMeasuredState();
    decltype(0_mps) GetMeasuredVelocity();
    void TestingVoltage();

private:
    std::unique_ptr<WPI_TalonFX> m_throttleMotor;
    std::unique_ptr<WPI_TalonFX> m_steeringMotor;

    units::radian_t m_steeringoffset;

    const frc::LinearSystem<1, 1, 1> m_throttleSystem = frc::LinearSystemId::IdentifyVelocitySystem<units::meter>(
        Constants::SwerveModule::throttleKv, Constants::SwerveModule::throttleKa);
    const frc::LinearSystem<2, 1, 1> m_steeringSystem = frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
        Constants::SwerveModule::steeringKv, Constants::SwerveModule::steeringKa);
    frc::LinearPlantInversionFeedforward<1, 1> m_throttleFeedforward{m_throttleSystem, Constants::LoopPeriod};
    frc::LinearPlantInversionFeedforward<2, 1> m_steeringFeedforward{m_steeringSystem, Constants::LoopPeriod};
};
