#include "commands/ClimbCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

WinchCommand::WinchCommand(double DesiredWinchPower, ClimbSubsystem& Climb)
  : m_climb(Climb), m_desiredWinchPower(DesiredWinchPower)
{
    AddRequirements(&Climb);
    SetName("Winch");
}

void WinchCommand::Execute()
{
    m_climb.WinchPull(m_desiredWinchPower);
}

bool WinchCommand::IsFinished()
{
    return true;
}

WinchReleaseCommand::WinchReleaseCommand(double DesiredWinchReleasePower, ClimbSubsystem& Climb)
  : m_climb(Climb), m_desiredWinchReleasePower(DesiredWinchReleasePower)
{
    // AddRequirements(&Climb);
}

void WinchReleaseCommand::Execute()
{
    m_climb.WinchRelease(m_desiredWinchReleasePower);
}

bool WinchReleaseCommand::IsFinished()
{
    return false;
}