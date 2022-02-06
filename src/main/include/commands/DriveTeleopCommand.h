#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrainSubsystem.h"

class DriveTeleopCommand : public frc2::CommandHelper<frc2::CommandBase, DriveTeleopCommand>
{
public:
    DriveTeleopCommand(
        DriveTrainSubsystem& drivetrain, frc::Joystick& translationJoystick, frc::Joystick& steeringJoystick);
    void Execute() override;

private:
    DriveTrainSubsystem& m_drivetrain;
    frc::Joystick& m_translationJoystick;
    frc::Joystick& m_steeringJoystick;
};