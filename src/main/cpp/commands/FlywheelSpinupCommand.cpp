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
    return false;
}

void FeederSpinupCommand::Initialize()
{
    m_feeder.StartFeeder(m_feederRPM);
}
bool FeederSpinupCommand::IsFinished()
{
    return false;
}
/*
TestSpinupCommand::TestSpinupCommand(double TestRPM, FlywheelSubsystem& Test) : m_testRPM(TestRPM), m_test(Test)
{
    AddRequirements(&Test);
    SetName("TestSpinupCommand");
}

void TestSpinupCommand::Initialize()
{
    m_test.TempHoodShooterTest(m_testRPM);
}

bool TestSpinupCommand::IsFinished()
{
    return false;
}
*/