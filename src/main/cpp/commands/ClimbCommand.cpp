/*
#include "commands/ClimbCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

TriggerCommand::TriggerCommand(ClimbSubsystem& Climb) : m_climb(Climb)
{
    AddRequirements(&Climb);
    SetName("Trigger");
}

void TriggerCommand::Execute()
{
    m_climb.TriggerRelease();
}

bool TriggerCommand::IsFinished()
{
    return true;
}

WinchCommand::WinchCommand(ClimbSubsystem& Climb) : m_climb(Climb)
{
    AddRequirements(&Climb);
    SetName("Winch");
}

void WinchCommand::Execute()
{
    if (m_climb.TriggerReleased() == true)
    {
        m_climb.WinchPull(Constants::Systemspeeds::WinchOutput);
    }
    else
    {
        m_climb.WinchPull(0);
    }
}

bool WinchCommand::IsFinished()
{
    return false;
}
*/