#include "subsystems/IntakeFeederSubsystem.h"

#include <frc2/command/button/JoystickButton.h>

#include "ctre/Phoenix.h"

IntakeFeederSubsystem::IntakeFeederSubsystem()
{
    m_intakeFeeder.ConfigFactoryDefault();

    m_intakeFeeder.SetNeutralMode(Brake);
}

void IntakeFeederSubsystem::Periodic()
{
}

void IntakeFeederSubsystem::StartIntakeFeeder(double IntakeFeederRpm)
{
    m_intakeFeeder.Set(ControlMode::PercentOutput, IntakeFeederRpm);
}