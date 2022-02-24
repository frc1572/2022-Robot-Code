#include "commands/FlywheelSpinupCommand.h"

#include <wpi/numbers>

FlywheelSpinupCommand::FlywheelSpinupCommand(rad_per_s_t omega, FlywheelSubsystem& flywheel)
  : m_omega(omega), m_flywheel(flywheel)
{
    AddRequirements(&flywheel);
}

FlywheelSpinupCommand::FlywheelSpinupCommand(double rpm, FlywheelSubsystem& flywheel)
  : FlywheelSpinupCommand(rpm * 2_rad * wpi::numbers::pi / 1_min, flywheel)
{
}

void FlywheelSpinupCommand::FeederSpinupCommand(double FeedRpm, FlywheelSubsystem& feeder) : m_feedRpm(FeedRpm)
{
    AddRequirements(&feeder);
}

void FlywheelSpinupCommand::Initialize()
{
    m_flywheel.SetSetpoint(m_omega);
    m_flywheel.StartFeeder(m_FeedRpm);
}

bool FlywheelSpinupCommand::IsFinished()
{
    // return m_flywheel.IsSpunup();
    return false;
}