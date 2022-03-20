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

TurretManual180::TurretManual180(TurretSubsystem& turret) : m_turret(turret)
{
    AddRequirements(&m_turret);
}

void TurretManual180::Initialize()
{
    m_turret.SetDesiredPosition(180_deg);
}

bool TurretManual180::IsFinished()
{
    return false;
}