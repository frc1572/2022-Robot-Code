#include "commands/VisionTurretCommand.h"

VisionTurretCommand::VisionTurretCommand(TurretSubsystem& turret, VisionSubsystem& limelight)
  : m_turret(turret), m_limelight(limelight)
{
}

void VisionTurretCommand::Execute()
{
    if (auto targetInfo = m_limelight.GetLatestResult())
    {
        // Only update if we have a target
        m_turret.SetDesiredPosition(m_turret.GetMeasuredPosition() + targetInfo->yaw);
    }
}