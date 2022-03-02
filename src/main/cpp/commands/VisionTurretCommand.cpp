#include "commands/VisionTurretCommand.h"

#include <iostream>

VisionTurretCommand::VisionTurretCommand(TurretSubsystem& turret, VisionSubsystem& limelight)
  : m_turret(turret), m_limelight(limelight)
{
}

void VisionTurretCommand::Execute()
{
    // std::cout << "Command Executing" << std::endl;
    // VisionSubsystem::TargetInfo = m_limelight.GetLatestResult();
    // m_turret.SetDesiredPosition(m_turret.GetMeasuredPosition() + targetInfo->yaw);
    // std::cout << "Command code Executing" << std::endl;

    if (auto TargetInfo = m_limelight.GetLatestResult())
    {
        // Only update if we have a target
        m_turret.SetDesiredPosition(m_turret.GetMeasuredPosition() + TargetInfo->yaw);
        std::cout << "Command code Executing" << std::endl;
    }
}
