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


/*
SetHoodSpeed::SetHoodSpeed(double SelectedHoodSpeed) : m_selectedHoodSpeed(SelectedHoodSpeed) {

}
*/
void FlywheelSpinupCommand::Initialize()
{
    m_flywheel.SetSetpoint(m_omega);
}

bool FlywheelSpinupCommand::IsFinished()
{
    return false;
}


