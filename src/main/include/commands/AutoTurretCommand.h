#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"

class AutoTurretCommand : public frc2::CommandHelper<frc2::CommandBase, AutoTurretCommand>
{
public:
    AutoTurretCommand(DriveTrainSubsystem& drivetrain, TurretSubsystem& turret, FlywheelSubsystem& flywheel);
    void Initialize() override;
    void Execute() override;

private:
    DriveTrainSubsystem& m_drivetrain;
    TurretSubsystem& m_turret;
    FlywheelSubsystem& m_flywheel;

    frc::Rotation2d m_previousDesiredAngle;
    frc::Pose2d m_previousPose;

    frc::Rotation2d CalculateDesiredAngle();
};