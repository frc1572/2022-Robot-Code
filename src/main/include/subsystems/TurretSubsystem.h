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
    void GetMotorVoltageSpike();
    frc::Rotation2d GetMeasuredPosition();
<<<<<<< Updated upstream
    void SetDesiredPosition(units::radian_t DesiredPosition);
    void SetAbsolutePosition(double AbsolutePositon);
    // void MoveTurret();  //Maybe used idk????
private:
    WPI_TalonFX m_turret{Constants::Turret::TurretPort};
    units::radian_t m_desiredPosition;
=======
    void SetDesiredPosition(frc::Rotation2d desiredPosition);
    void Periodic() override;
    // void AddDesiredPosition(units::radian_t positionOffset);
    // void Periodic() override;

private:
    WPI_TalonFX m_turret{Constants::Turret::TurretPort};
    double turretFeedForward;
    units::radian_t m_turretOffset;
    frc::Rotation2d m_desiredPosition;
>>>>>>> Stashed changes

    const frc::LinearSystem<2, 1, 1> m_turretSystem = frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
        Constants::Turret::TurretKv, Constants::Turret::TurretKa);
    frc::LinearPlantInversionFeedforward<2, 1> m_turretFeedForward{m_turretSystem, Constants::LoopPeriod};
};
