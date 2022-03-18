#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ClimbSubsystem.h"

class WinchCommand : public frc2::CommandHelper<frc2::CommandBase, WinchCommand>
{
public:
    WinchCommand(double DesiredWinchPower, ClimbSubsystem& m_climb);
    void Execute() override;
    bool IsFinished() override;

private:
    ClimbSubsystem& m_climb;
    double m_desiredWinchPower;
};

class WinchReleaseCommand : public frc2::CommandHelper<frc2::CommandBase, WinchReleaseCommand>
{
public:
    WinchReleaseCommand(double DesiredWinchReleasePower, ClimbSubsystem& m_climb);
    void Execute() override;
    bool IsFinished() override;

private:
    ClimbSubsystem& m_climb;
    double m_desiredWinchReleasePower;
};