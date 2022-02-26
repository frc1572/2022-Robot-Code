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

// Setting up the FeederSpinupCommand VVV
FeederSpinupCommand::FeederSpinupCommand(double FeedRPM, FlywheelSubsystem& feeder)
  : m_feederRPM(FeedRPM), m_feeder(feeder)
{
    AddRequirements(&feeder);
}

MainFeederSpinupCommand::MainFeederSpinupCommand(double MainFeederRPM, FlywheelSubsystem& mainFeeder)
  : m_mainFeederRPM(MainFeederRPM), m_mainFeeder(mainFeeder)
{
    AddRequirements(&mainFeeder);
}

IntakeSpinupCommand::IntakeSpinupCommand(double IntakeRPM, FlywheelSubsystem& intake)
  : m_intakeRPM(IntakeRPM), m_intake(intake)
{
    AddRequirements(&intake);
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

void MainFeederSpinupCommand::Initialize()
{
    m_mainFeeder.StartMainFeeder(m_mainFeederRPM);
}
bool MainFeederSpinupCommand::IsFinished()
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