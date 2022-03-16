#pragma once 

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/Servo.h>
#include "Constants.h"

class ClimbSubsystem :  public frc2::SubsystemBase {
public:
    ClimbSubsystem();
    void WinchPull();
    void TriggerRelease();
    double GetTriggerPosition();
private: 
    WPI_TalonFX Winch{Constants::Climb::WinchID, "canviore"};
    frc::Servo ClimbTrigger{Constants::Climb::TriggerReleaseID};
};