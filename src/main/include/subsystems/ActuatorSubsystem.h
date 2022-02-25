#pragma once

#include <ctre/Phoenix.h>
#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

#include "frc/Joystick.h"

class ActuatorSubsystem : public frc2::SubsystemBase
{
public:
    ActuatorSubsystem(/*int LeftActuatorPort, int RightActuatorPort*/);
    void Periodic() override;
    void SetActuatorPosition(double TargetPosition);
    // void SetActuatorPoistion();

private:
    // Working on makeing the system take in one port and create teo objects for servos <<< Using one physical port to
    // controll both actuators
    frc::Servo m_actuators{1};
    // frc::Servo m_leftActuator{1};
    // frc::Servo m_rightActuator{2};
    //  std::unique_ptr<frc::Servo> m_leftActuator;
    //  std::unique_ptr<frc::Servo> m_rightActuator;
};
