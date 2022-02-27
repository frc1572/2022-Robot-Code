#include "commands/ActuatorCommand.h"

#include <spdlog/spdlog.h>

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
    spdlog::info(">>>>>IsFinished Command Ran<<<<<");
    return true;
}