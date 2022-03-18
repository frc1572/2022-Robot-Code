#include "commands/ClimbCommand.h"

#include <iostream>

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
    std::cout << "Excecute Is running" << std::endl;
    if (m_translationJoystick.GetRawButton(1) == true && m_joystick.GetRawButton(12) == true)
    {
        m_climb.WinchPull(m_desiredWinchPower);
        std::cout << "Winch Pull Command is Running" << std::endl;
    }
    else
    {
        m_climb.WinchPull(0.0);
        std::cout << "Winch Pull Command is NOT Running" << std::endl;
    }
}

bool WinchCommand::IsFinished()
{
    return false;
}
