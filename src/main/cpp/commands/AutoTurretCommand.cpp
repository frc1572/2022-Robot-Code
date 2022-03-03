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
    auto goalOffset = Constants::GoalTranslation - m_drivetrain.GetPose().Translation();
    auto angleOffset =
        frc::Rotation2d(units::math::atan2(goalOffset.Y(), goalOffset.X())) - m_drivetrain.GetMeasuredRotation();
    // spdlog::info("{}", angleOffset.Degrees().value());
    m_turret.SetDesiredPosition(angleOffset);
}
