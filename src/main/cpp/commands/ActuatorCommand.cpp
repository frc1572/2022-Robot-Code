#include "commands/ActuatorCommand.h"

#include <iostream>

ActuatorCommand::ActuatorCommand(double ActuatorPosition, ActuatorSubsystem& Actuators)
  : m_actuatorPosition(ActuatorPosition), m_actuators(Actuators)
{
    AddRequirements(&Actuators);
}

void ActuatorCommand::Initialize()
{
    // Command for Actuators
    m_actuators.SetActuatorPosition(m_actuatorPosition);
}

bool ActuatorCommand::IsFinished()
{
    std::cout << ">>>>>IsFinished Command Ran<<<<<" << std::endl;
    return true;
}