#include "commands/AutoTurretCommand.h"

#include <spdlog/spdlog.h>

AutoTurretCommand::AutoTurretCommand(DriveTrainSubsystem& drivetrain, TurretSubsystem& turret)
  : m_drivetrain(drivetrain), m_turret(turret)
{
    AddRequirements(&m_turret);
    SetName("AutoTurretCommand");
}

void AutoTurretCommand::Initialize()
{
    m_previousDesiredAngle = CalculateDesiredAngle();
}

void AutoTurretCommand::Execute()
{
    auto desiredAngle = CalculateDesiredAngle();
    auto desiredVelocity = (desiredAngle.Radians() - m_previousDesiredAngle.Radians()) / Constants::LoopPeriod;
    m_turret.SetDesiredPosition(desiredAngle, desiredVelocity);
    m_previousDesiredAngle = desiredAngle;
}

frc::Rotation2d AutoTurretCommand::CalculateDesiredAngle()
{
    auto pose = m_drivetrain.GetPose();
    auto goalOffset = Constants::GoalTranslation - pose.Translation();
    return frc::Rotation2d(units::math::atan2(goalOffset.Y(), goalOffset.X())) - pose.Rotation();
}