#include "commands/TurretCommand.h"

TurretCommand::TurretCommand(TurretSubsystem& Turret, frc::Joystick& TurretJoystick)
  : m_turret(Turret), m_turretJoystick(TurretJoystick)
{
    AddRequirements(&m_turret);
};

void TurretCommand::Execute()
{
    // TODO: Add deadband

    double PositionX = m_turretJoystick.GetX();

    // m_turret.SetDesiredPosition(PositionX * Constants::Turret::TurretMaxVelocity);
    m_turret.SetDesiredPosition(0_deg);
}