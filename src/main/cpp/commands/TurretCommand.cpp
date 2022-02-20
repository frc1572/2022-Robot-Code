#include "commands/TurretCommand.h"

TurretCommand::TurretCommand(TurretSubsystem& Turret, frc::Joystick& TurretJoystick)
  : m_turret(Turret), m_turretJoystick(TurretJoystick)
{
    AddRequirements(&m_turret);
};

void TurretCommand::Execute()
{
    // TODO: Add deadband
    m_turret.AddDesiredPosition(m_turretJoystick.GetX() * Constants::Turret::TurretMaxVelocity * Constants::LoopPeriod);
}