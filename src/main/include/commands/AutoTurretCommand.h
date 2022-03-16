#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/TurretSubsystem.h"

class AutoTurretCommand : public frc2::CommandHelper<frc2::CommandBase, AutoTurretCommand>
{
public:
    AutoTurretCommand(DriveTrainSubsystem& drivetrain, TurretSubsystem& turret);
    void Initialize() override;
    void Execute() override;

private:
    DriveTrainSubsystem& m_drivetrain;
    TurretSubsystem& m_turret;

    frc::Rotation2d m_previousDesiredAngle;

    frc::Rotation2d CalculateDesiredAngle();
};