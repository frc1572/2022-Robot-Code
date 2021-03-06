#pragma once

#include <subsystems/VisionSubsystem.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/TurretSubsystem.h"

class TurretManualControl : public frc2::CommandHelper<frc2::CommandBase, TurretManualControl>
{
public:
    TurretManualControl(TurretSubsystem& turret);
    void Initialize() override;
    bool IsFinished() override;

private:
    TurretSubsystem& m_turret;
};

class TurretManual180 : public frc2::CommandHelper<frc2::CommandBase, TurretManual180>
{
public:
    TurretManual180(double DesiredRotation, TurretSubsystem& turret);
    void Initialize() override;
    bool IsFinished() override;

private:
    TurretSubsystem& m_turret;
    double m_desiredRotation;
};