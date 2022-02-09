#include "commands/FlywheelSpinupCommand.h"

#include <wpi/numbers>

FlywheelSpinupCommand::FlywheelSpinupCommand(rad_per_s_t omega, FlywheelSubsystem& flywheel)
  : m_omega(omega), m_flywheel(flywheel)
{
    AddRequirements(&flywheel);
}

FlywheelSpinupCommand::FlywheelSpinupCommand(double rpm, FlywheelSubsystem& flywheel)
  : FlywheelSpinupCommand(rpm * 1_tr / 1_min, flywheel)
{
}

void FlywheelSpinupCommand::Initialize()
{
    m_flywheel.SetSetpoint(m_omega);
}

bool FlywheelSpinupCommand::IsFinished()
{
    // return m_flywheel.IsSpunup();
    return false;
}