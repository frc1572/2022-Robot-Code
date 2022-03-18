#include "commands/ClimbCommand.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

WinchCommand::WinchCommand(
    double DesiredWinchPower, ClimbSubsystem& Climb, frc::Joystick& TranslationJoystick, frc::Joystick& Joystick)
  : m_climb(Climb), m_desiredWinchPower(DesiredWinchPower), m_translationJoystick(TranslationJoystick),
    m_joystick(Joystick)
{
    AddRequirements(&Climb);
    SetName("Winch");
}

void WinchCommand::Execute()
{
    if (m_translationJoystick.GetRawButton(1) == true && m_joystick.GetRawButton(12) == true)
    {
        m_climb.WinchPull(m_desiredWinchPower);
    }
    else
    {
        m_climb.WinchPull(0.0);
    }
}

bool WinchCommand::IsFinished()
{
    return false;
}
