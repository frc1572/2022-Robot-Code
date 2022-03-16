#include "commands/ClimbCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

TriggerCommand::TriggerCommand(ClimbSubsystem& Climb) : m_climb(Climb){
    AddRequirements(&Climb);
    SetName("Trigger");
}

bool TriggerCommand::TriggerReleased() {
    if (m_climb.GetTriggerPosition() != Constants::Climb::TriggerHoldPosition) {
        return true;
    }
    else {
        return false;
    }
}

void TriggerCommand::Execute() {
    m_climb.TriggerRelease();
}

bool TriggerCommand::IsFinished() {
    return true;
}

WinchCommand::WinchCommand(ClimbSubsystem& Climb) : m_climb(Climb){
    AddRequirements(&Climb);
    SetName("Winch");
}

void WinchCommand::Execute() {
    if (TriggerCommand::TriggerReleased() == true) {
        m_climb.WinchPull(Constants::Systemspeeds::WinchOutput);
    }
    else {
        m_climb.WinchPull(0);
    }
}

bool WinchCommand::IsFinished() {
    return false;
}