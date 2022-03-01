#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Rotation2d.h>

#include "CustomUnits.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class TurretCommand : public frc2::CommandHelper<frc2::CommandBase, TurretCommand>
{
public:
    TurretCommand(frc::Rotation2d PlannedTurretAngle, TurretSubsystem& turret, VisionSubsystem& limelight);
    void Execute() override;
    bool IsFinished() override;

private:
    frc::Rotation2d m_plannedTurretAngle;
    TurretSubsystem& m_turret;
    VisionSubsystem& m_limelight;
};