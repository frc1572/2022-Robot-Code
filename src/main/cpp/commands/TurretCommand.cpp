#include "commands/TurretCommand.h"

#include <frc2/command/button/JoystickButton.h>

TurretCommand::TurretCommand(frc::Rotation2d PlannedTurretAngle, TurretSubsystem& turret)
  : m_plannedTurretAngle(PlannedTurretAngle), m_turret(turret)
{
    // AddRequirements(&m_turret);
}

void TurretCommand::Execute()
{
    m_turret.SetDesiredPosition(m_plannedTurretAngle);
    // Constants::Turret::Turretposition * 1.0
}

bool TurretCommand::IsFinished()
{
    return true;
}