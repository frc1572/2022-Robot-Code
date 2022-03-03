#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "Constants.h"

class TurretSubsystem : public frc2::SubsystemBase
{
public:
    TurretSubsystem();
    // void GetCurrent();
    frc::Rotation2d GetMeasuredPosition();
    void SetDesiredPosition(frc::Rotation2d desiredPosition);
    decltype(1_rad_per_s) GetMeasuredVelocity();
    void Periodic() override;
    void SimulationPeriodic() override;

private:
    WPI_TalonFX m_turret{Constants::Turret::TurretPort};

    units::radian_t m_turretOffset = 0_deg; // 1_deg;
    frc::Rotation2d m_desiredPosition;

    const frc::LinearSystem<2, 1, 1> m_turretSystem = frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
        Constants::Turret::TurretKv, Constants::Turret::TurretKa);
    frc::LinearPlantInversionFeedforward<2, 1> m_turretFeedForward{m_turretSystem, Constants::LoopPeriod};

    frc::sim::LinearSystemSim<2, 1, 1> m_turretSim{m_turretSystem, {0.0}};
};
