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
        frc::Rotation2d(units::math::atan2(goalOffset.Y(), goalOffset.X())) - m_drivetrain.GetMeasuredRotation();
    m_turret.SetDesiredPosition(angleOffst);
    std::cout << "GoalOffset x: " << goalOffset.X().value() << "   GoalOffset y: " << goalOffset.Y().value()
              << "   DriveTrain Rotation: " << m_drivetrain.GetMeasuredRotation().Degrees().value() << std::endl;
}
