#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ActuatorSubsystem.h"

class ActuatorCommand : public frc2::CommandHelper<frc2::CommandBase, ActuatorCommand>
{
public:
    ActuatorCommand(double ActuatorPosition, ActuatorSubsystem& Actuators);
    void Initialize() override;
    bool IsFinished() override;

private:
    ActuatorSubsystem& m_actuators;
    double m_actuatorPosition;
    // frc::Joystick& m_joystick;
};