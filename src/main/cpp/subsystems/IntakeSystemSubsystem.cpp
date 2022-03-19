#include "subsystems/IntakeSystemSubsystem.h"

#include <frc2/command/button/JoystickButton.h>

#include "ctre/Phoenix.h"

IntakeSystemSubsystem::IntakeSystemSubsystem()
{
    m_intake.ConfigFactoryDefault();

    m_intake.SetNeutralMode(Coast);
}

void IntakeSystemSubsystem::Periodic()
{
}

void IntakeSystemSubsystem::StartIntake(double IntakeRpm)
{
    m_intake.Set(ControlMode::PercentOutput, IntakeRpm);
}