#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

#include "Constants.h"

class TurretSubsystem : public frc2::SubsystemBase
{
public:
    TurretSubsystem();
    // void GetCurrent();
    frc::Rotation2d GetMeasuredPosition();
    void SetDesiredPosition(double desiredPosition);
    // void AddDesiredPosition(units::radian_t positionOffset);
    // void Periodic() override;

private:
    WPI_TalonFX m_turret{Constants::Turret::TurretPort};

    units::radian_t m_turretOffset;

    const frc::LinearSystem<2, 1, 1> m_turretSystem = frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
        Constants::Turret::TurretKv, Constants::Turret::TurretKa);
    frc::LinearPlantInversionFeedforward<2, 1> m_turretFeedForward{m_turretSystem, Constants::LoopPeriod};
};
