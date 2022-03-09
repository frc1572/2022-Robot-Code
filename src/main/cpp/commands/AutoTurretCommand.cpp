#include "commands/AutoTurretCommand.h"

#include <spdlog/spdlog.h>

AutoTurretCommand::AutoTurretCommand(DriveTrainSubsystem& drivetrain, TurretSubsystem& turret)
  : m_drivetrain(drivetrain), m_turret(turret)
{
    AddRequirements(&m_turret);
    SetName("AutoTurretCommand");
}

void AutoTurretCommand::Execute()
{
    auto pose = m_drivetrain.GetPose();
    auto goalOffset = Constants::GoalTranslation - pose.Translation();
    auto angleOffset = frc::Rotation2d(units::math::atan2(goalOffset.Y() * -1, goalOffset.X())) - pose.Rotation();

    // sdplog::info("{}", angleOffset.Degrees().value());
    m_turret.SetDesiredPosition(angleOffset);
}
