#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <photonlib/PhotonCamera.h>

#include "CustomUnits.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class TurretCommand : public frc2::CommandHelper<frc2::CommandBase, TurretCommand>
{
public:
    TurretCommand(double PlannedTurretAngle, TurretSubsystem& turret, VisionSubsystem& limelight);
    void Execute() override;
    bool IsFinished() override;

private:
    double m_plannedTurretAngle;
    TurretSubsystem& m_turret;
    VisionSubsystem& m_limelight;
};