/*
#include "subsystems/ClimbSubsystem.h"

ClimbSubsystem::ClimbSubsystem()
{
}

void ClimbSubsystem::WinchPull(double winchPower)
{
    Winch.Set(ControlMode::PercentOutput, winchPower);
}

void ClimbSubsystem::TriggerRelease()
{
    ClimbTrigger.Set(Constants::Climb::TriggerReleasePosition);
}

bool ClimbSubsystem::TriggerReleased()
{
    if (ClimbTrigger.Get() != Constants::Climb::TriggerHoldPosition)
    {
        return true;
    }
    else
    {
        return false;
    }
}
*/