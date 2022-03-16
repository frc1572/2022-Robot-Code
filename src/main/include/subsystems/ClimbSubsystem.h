/*
#pragma once

#include <ctre/Phoenix.h>
#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ClimbSubsystem : public frc2::SubsystemBase
{
public:
    ClimbSubsystem();
    void WinchPull(double winchPower);
    void TriggerRelease();
    bool TriggerReleased();

private:
    WPI_TalonFX Winch{Constants::Climb::WinchID, "canviore"};
    frc::Servo ClimbTrigger{Constants::Climb::TriggerReleaseID};
};
*/