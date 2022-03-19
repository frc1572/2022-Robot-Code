#include "commands/TurretManualControl.h"

TurretManualControl::TurretManualControl(TurretSubsystem& turret) : m_turret(turret)
{
    AddRequirements(&m_turret);
}

void TurretManualControl::Initialize()
{
    auto turretCurrentPose = m_turret.GetMeasuredRotation();
    m_turret.SetDesiredPosition(turretCurrentPose);
}

bool TurretManualControl::IsFinished()
{
    return false;
}