#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "CustomUnits.h"
#include "subsystems/TurretSubsystem.h"

class TurretCommand : public frc2::CommandHelper<frc2::CommandBase, TurretCommand>
{
public:
    TurretCommand(TurretSubsystem& Turret, frc::Joystick& TurretJoystick);
    void Execute() override;

private:
    TurretSubsystem& m_turret;
    frc::Joystick& m_turretJoystick;
};