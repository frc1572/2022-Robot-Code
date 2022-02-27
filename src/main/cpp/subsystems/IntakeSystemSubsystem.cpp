#include "subsystems/IntakeSystemSubsystem.h"

#include <frc2/command/button/JoystickButton.h>

#include "ctre/Phoenix.h"

IntakeSystemSubsystem::IntakeSystemSubsystem()
{
    m_intake.ConfigFactoryDefault();
    m_intakeFeeder.ConfigFactoryDefault();

    m_intake.SetNeutralMode(Coast);
    m_intakeFeeder.SetNeutralMode(Brake);
}

void IntakeSystemSubsystem::Periodic()
{
}

void IntakeSystemSubsystem::StartIntakeFeeder(double IntakeFeederRpm)
{
    m_intakeFeeder.Set(ControlMode::PercentOutput, IntakeFeederRpm);
}

void IntakeSystemSubsystem::StartIntake(double IntakeRpm)
{
    m_intake.Set(ControlMode::PercentOutput, IntakeRpm);
}