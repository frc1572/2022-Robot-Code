#pragma once

<<<<<<< Updated upstream
#include <frc/Joystick.h>
=======
#include <frc/geometry/Rotation2d.h>
>>>>>>> Stashed changes
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "CustomUnits.h"
#include "subsystems/TurretSubsystem.h"

class TurretCommand : frc2::CommandHelper<frc2::CommandBase, TurretCommand>
{
public:
<<<<<<< Updated upstream
    TurretCommand(TurretSubsystem& Turret, frc::Joystick& TurretJoystick);
=======
    TurretCommand(frc::Rotation2d PlannedTurretAngle, TurretSubsystem& turret /*, VisionSubsystem& limelight*/);
>>>>>>> Stashed changes
    void Execute() override;

private:
    TurretSubsystem& m_turret;
<<<<<<< Updated upstream
    frc::Joystick& m_turretJoystick;
=======
    // VisionSubsystem& m_limelight;
>>>>>>> Stashed changes
};