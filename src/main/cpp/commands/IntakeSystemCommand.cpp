#include "commands/IntakeSystemCommand.h"

IntakeFeederSpinupCommand::IntakeFeederSpinupCommand(double IntakeFeederRPM, IntakeSystemSubsystem& intakeFeeder)
  : m_intakeFeederRPM(IntakeFeederRPM), m_intakeFeeder(intakeFeeder)
{
    AddRequirements(&intakeFeeder);
}

IntakeSpinupCommand::IntakeSpinupCommand(double IntakeRPM, IntakeSystemSubsystem& intake)
  : m_intakeRPM(IntakeRPM), m_intake(intake)
{
    AddRequirements(&intake);
}

void IntakeFeederSpinupCommand::Initialize()
{
    m_intakeFeeder.StartIntakeFeeder(m_intakeFeederRPM);
}
bool IntakeFeederSpinupCommand::IsFinished()
{
    return false;
}

void IntakeSpinupCommand::Initialize()
{
    m_intake.StartIntake(m_intakeRPM);
}
bool IntakeSpinupCommand::IsFinished()
{
    return false;
}