#include "commands/FlywheelSpinupCommand.h"

#include <wpi/numbers>

FlywheelSpinupCommand::FlywheelSpinupCommand(rad_per_s_t omega, FlywheelSubsystem& flywheel)
  : m_omega(omega), m_flywheel(flywheel)
{
    AddRequirements(&flywheel);
    SetName("FlywheelSpinupCommand");
}

FlywheelSpinupCommand::FlywheelSpinupCommand(double rpm, FlywheelSubsystem& flywheel)
  : FlywheelSpinupCommand(rpm * 2_rad * wpi::numbers::pi / 1_min, flywheel)
{
}

void FlywheelSpinupCommand::Initialize()
{
    // m_flywheel.SetSetpoint(m_omega);
}

void FlywheelSpinupCommand::Execute()
{
    m_flywheel.SetSetpoint(
        frc::SmartDashboard::GetNumber("Desired Flywheel RPM:", 0.0) * 2_rad * wpi::numbers::pi / 1_min);
}

void FlywheelSpinupCommand::End(bool interrupted)
{
    m_flywheel.SetSetpoint(0.0 * 2_rad * wpi::numbers::pi / 1_min);
}

bool FlywheelSpinupCommand::IsFinished()
{
    return false;
}
