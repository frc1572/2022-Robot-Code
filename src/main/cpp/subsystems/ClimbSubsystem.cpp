#include "subsystems/ClimbSubsystem.h"

ClimbSubsystem::ClimbSubsystem() {
}

void ClimbSubsystem::WinchPull() {
    Winch.Set(ControlMode::PercentOutput, Constants::Systemspeeds::WinchOutput);
}

void ClimbSubsystem::TriggerRelease() {
    ClimbTrigger.Set(Constants::Climb::TriggerReleasePosition);
}

double ClimbSubsystem::GetTriggerPosition() {
    return ClimbTrigger.Get();
}