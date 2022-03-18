#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ClimbSubsystem.h"

class WinchCommand : public frc2::CommandHelper<frc2::CommandBase, WinchCommand>
{
public:
    WinchCommand(
        double DesiredWinchPower, ClimbSubsystem& m_climb, frc::Joystick& TranslationJoystick, frc::Joystick& Joystick);
    void Execute() override;
    bool IsFinished() override;

private:
    ClimbSubsystem& m_climb;
    frc::Joystick& m_translationJoystick;
    frc::Joystick& m_joystick;
    double m_desiredWinchPower;
};
