#include "subsystems/ClimbSubsystem.h"

#include "frc2/command/WaitCommand.h"
ClimbSubsystem::ClimbSubsystem()
{
}

void ClimbSubsystem::WinchPull(double winchPower)
{
    m_winch.Set(ControlMode::PercentOutput, winchPower);
}

void ClimbSubsystem::WinchRelease(double winchReleasePower)
{
    m_winch.Set(ControlMode::PercentOutput, winchReleasePower);
    frc2::WaitCommand(1.0_s);
    m_winch.Set(ControlMode::PercentOutput, 0.0);
}