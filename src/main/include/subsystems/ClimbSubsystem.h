
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
    void WinchRelease(double WinchReleasePower);

private:
    WPI_TalonFX m_winch{Constants::Climb::WinchID, "canviore"};
};
