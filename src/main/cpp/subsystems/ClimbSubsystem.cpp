#include "subsystems/ClimbSubsystem.h"

#include "frc2/command/WaitCommand.h"
ClimbSubsystem::ClimbSubsystem()
{
    m_winch.ConfigFactoryDefault();
    m_winch.SetNeutralMode(Coast);
    m_winch.SetInverted(true);
}

void ClimbSubsystem::WinchPull(double winchPower)
{
    m_winch.Set(ControlMode::PercentOutput, winchPower);
}
