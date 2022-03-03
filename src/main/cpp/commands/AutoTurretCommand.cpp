#include "commands/AutoTurretCommand.h"

#include <iostream>

AutoTurretCommand::AutoTurretCommand(DriveTrainSubsystem& drivetrain, TurretSubsystem& turret)
  : m_drivetrain(drivetrain), m_turret(turret)
{
    AddRequirements(&m_turret);
}

void AutoTurretCommand::Execute()
{
    auto goalOffset = Constants::GoalTranslation - m_drivetrain.GetPose().Translation();
    auto angleOffst =
        frc::Rotation2d(units::math::atan2(goalOffset.X(), goalOffset.Y())) - m_drivetrain.GetMeasuredRotation();
    m_turret.SetDesiredPosition(angleOffst);
}
