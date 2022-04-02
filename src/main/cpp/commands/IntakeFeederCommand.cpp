#include "commands/IntakeFeederCommand.h"

#include "frc/smartdashboard/SmartDashboard.h"
IntakeFeederCommand::IntakeFeederCommand(double IntakeFeederRPM, IntakeFeederSubsystem& intakeFeeder)
  : m_intakeFeederRPM(IntakeFeederRPM), m_intakeFeeder(intakeFeeder)
{
    AddRequirements(&m_intakeFeeder);
    SetName("IntakeFeederSpinupCommand");
}

void IntakeFeederCommand::Initialize()
{
    // m_intakeFeeder.StartIntakeFeeder(m_intakeFeederRPM);
}
void IntakeFeederCommand::Execute()
{
    m_intakeFeeder.StartIntakeFeeder(
        frc::SmartDashboard::GetNumber("Conveyor Speed (0-1): ", 0) /* m_intakeFeederRPM*/);
}

void IntakeFeederCommand::End(bool interrupted)
{
    m_intakeFeeder.StartIntakeFeeder(0);
}

bool IntakeFeederCommand::IsFinished()
{
    return false;
}
