#pragma once

#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

#include "frc/Joystick.h"

class ActuatorSubsystem : public frc2::SubsystemBase
{
public:
    ActuatorSubsystem();
    void Periodic() override;
    void SetActutorPoistion();

private:
    frc::Servo m_left{1};
    frc::Servo m_right{2};
    frc::Joystick m_ActuatorJoystick{0};
};
