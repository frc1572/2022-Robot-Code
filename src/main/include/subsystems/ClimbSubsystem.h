
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

private:
    WPI_TalonFX m_winch{Constants::Climb::WinchID, "canivore"};
};
