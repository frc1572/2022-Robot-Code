#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <photonlib/PhotonCamera.h>

#include "subsystems/TurretSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class VisionTurretCommand : public frc2::CommandHelper<frc2::CommandBase, VisionTurretCommand>
{
public:
    VisionTurretCommand(TurretSubsystem& turret, VisionSubsystem& limelight);
    void Execute() override;

private:
    TurretSubsystem& m_turret;
    VisionSubsystem& m_limelight;
};