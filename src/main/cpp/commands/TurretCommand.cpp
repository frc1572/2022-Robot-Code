#include "commands/TurretCommand.h"

#include <frc2/command/button/JoystickButton.h>

TurretCommand::TurretCommand(double PlannedTurretAngle, TurretSubsystem& turret, VisionSubsystem& limelight)
  : m_plannedTurretAngle(PlannedTurretAngle), m_turret(turret), m_limelight(limelight)
{
    AddRequirements(&m_turret);
    SetName("TurretCommand");
}

void TurretCommand::Execute()
{
    m_turret.SetDesiredPosition(m_plannedTurretAngle);
    // Constants::Turret::Turretposition * 1.0
}

/*
void TurretCommand::Execute()
{
    m_turret.SetDesiredPosition(m_plannedTurretAngle);
    // TODO: Add deadband
    // m_turret.AddDesiredPosition(.GetX() * Constants::Turret::TurretMaxVelocity * Constants::LoopPeriod);
}
*/

bool TurretCommand::IsFinished()
{
    return false;
}