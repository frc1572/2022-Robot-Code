#include "commands/TurretCommand.h"

<<<<<<< Updated upstream
TurretCommand::TurretCommand(TurretSubsystem& Turret, frc::Joystick& TurretJoystick)
  : m_turret(Turret), m_turretJoystick(TurretJoystick)
=======
#include <frc2/command/button/JoystickButton.h>

TurretCommand::TurretCommand(
    frc::Rotation2d PlannedTurretAngle, TurretSubsystem& turret /*, VisionSubsystem& limelight*/)
  : m_plannedTurretAngle(PlannedTurretAngle), m_turret(turret), // m_limelight(limelight)
>>>>>>> Stashed changes
{
    AddRequirements(&m_turret);
};

void TurretCommand::Execute()
{
    // TODO: Add deadband

    double PositionX = m_turretJoystick.GetX();

    m_turret.SetDesiredPosition(PositionX * Constants::Turret::TurretMaxVelocity);
}