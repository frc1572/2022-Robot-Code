#pragma once

#include <ctre/Phoenix.h>
#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

#include "frc/Joystick.h"

class ActuatorSubsystem : public frc2::SubsystemBase
{
public:
    ActuatorSubsystem();
    void Periodic() override;
    void SetActuatorPosition(double TargetPosition);

private:
    frc::Servo m_actuators{1};
};
