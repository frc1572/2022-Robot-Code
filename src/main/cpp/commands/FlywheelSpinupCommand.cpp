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

// Setting up the FeederSpinupCommand VVV
FeederSpinupCommand::FeederSpinupCommand(double FeedRPM, FlywheelSubsystem& feeder)
  : m_feederRPM(FeedRPM), m_feeder(feeder)
{
    AddRequirements(&feeder);
    SetName("FeederSpinupCommand");
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

// Feeder Command Initialize and IsFinished VVV
void FeederSpinupCommand::Initialize()
{
    m_feeder.StartFeeder(m_feederRPM);
}
bool FeederSpinupCommand::IsFinished()
{
    return false;
}
