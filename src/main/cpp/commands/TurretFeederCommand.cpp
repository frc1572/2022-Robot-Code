#include "commands/TurretFeederCommand.h"

#include <wpi/numbers>

FeederSpinupCommand::FeederSpinupCommand(double FeedRPM, TurretFeederSubsystem& feeder)
  : m_feederRPM(FeedRPM), m_feeder(feeder)
{
    AddRequirements(&feeder);
    SetName("FeederSpinupCommand");
}

void FeederSpinupCommand::Initialize()
{
    m_feeder.StartFeeder(m_feederRPM);
}
bool FeederSpinupCommand::IsFinished()
{
    return false;
}
