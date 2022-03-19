#include "commands/IntakeSystemCommand.h"

IntakeSpinupCommand::IntakeSpinupCommand(double IntakeRPM, IntakeSystemSubsystem& intake)
  : m_intakeRPM(IntakeRPM), m_intake(intake)
{
    AddRequirements(&intake);
    SetName("IntakeSpinupCommand");
}

void IntakeSpinupCommand::Initialize()
{
    m_intake.StartIntake(m_intakeRPM);
}
bool IntakeSpinupCommand::IsFinished()
{
    return false;
}