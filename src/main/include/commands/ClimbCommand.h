#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimbSubsystem.h"

class TriggerCommand : public frc2::CommandHelper<frc2::CommandBase, TriggerCommand> {
public:
    TriggerCommand(ClimbSubsystem& Climb);
    bool TriggerReleased();
    void Execute() override;
    bool IsFinished() override;
private:
    ClimbSubsystem& m_climb;
};

class WinchCommand : public frc2::CommandHelper<frc2::CommandBase, WinchCommand> {
public:
    WinchCommand(ClimbSubsystem& m_climb);
    void Execute() override;
    bool IsFinished() override;
private:
    ClimbSubsystem& m_climb;
};