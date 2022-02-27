#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "CustomUnits.h"
#include "subsystems/TurretSubsystem.h"

class TurretCommand : public frc2::CommandHelper<frc2::CommandBase, TurretCommand>
{
public:
    TurretCommand(double PlannedTurretAngle, TurretSubsystem& turret);
    // void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    double m_plannedTurretAngle;
    TurretSubsystem& m_turret;
};