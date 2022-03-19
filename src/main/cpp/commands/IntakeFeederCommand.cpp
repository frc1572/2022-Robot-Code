#include "commands/IntakeFeederCommand.h"

IntakeFeederCommand::IntakeFeederCommand(double IntakeFeederRPM, IntakeFeederSubsystem& intakeFeeder)
  : m_intakeFeederRPM(IntakeFeederRPM), m_intakeFeeder(intakeFeeder)
{
    AddRequirements(&m_intakeFeeder);
    SetName("IntakeFeederSpinupCommand");
}

void IntakeFeederCommand::Initialize()
{
    m_intakeFeeder.StartIntakeFeeder(m_intakeFeederRPM);
}
bool IntakeFeederCommand::IsFinished()
{
    return false;
}
