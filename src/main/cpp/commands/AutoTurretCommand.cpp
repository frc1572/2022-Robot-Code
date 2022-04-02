#include "commands/AutoTurretCommand.h"

#include <spdlog/spdlog.h>
#include <wpi/numbers>

AutoTurretCommand::AutoTurretCommand(
    DriveTrainSubsystem& drivetrain, TurretSubsystem& turret, FlywheelSubsystem& flywheel, VisionSubsystem& vision)
  : m_drivetrain(drivetrain), m_turret(turret), m_flywheel(flywheel), m_vision(vision)
{
    AddRequirements(&m_turret);
    SetName("AutoTurretCommand");
}

void AutoTurretCommand::Initialize()
{
    m_previousDesiredAngle = CalculateDesiredAngle();
    m_previousPose = m_drivetrain.GetPose();
}

void AutoTurretCommand::Execute()
{
    auto turretAngle = m_vision.GetLatestResult();
    auto desiredAngle = CalculateDesiredAngle() + turretAngle->yaw;
    auto desiredVelocity = (desiredAngle.Radians() - m_previousDesiredAngle.Radians()) / Constants::LoopPeriod;
    m_turret.SetDesiredPosition(desiredAngle, desiredVelocity);
    m_previousDesiredAngle = desiredAngle;
}

frc::Rotation2d AutoTurretCommand::CalculateDesiredAngle()
{
    auto pose = m_drivetrain.GetPose();
    auto goalOffset = Constants::GoalTranslation - pose.Translation();

    auto flywheelVelocity = m_flywheel.GetDesiredVelocity();
    if (flywheelVelocity > decltype(flywheelVelocity){0})
    {
        auto poseChange = pose.Translation() - m_previousPose.Translation();
        auto dx = poseChange.X() / Constants::LoopPeriod;
        auto dy = poseChange.Y() / Constants::LoopPeriod;
        auto flywheelTangentialVelocity = flywheelVelocity / 1_tr * Constants::Flywheel::Diameter * wpi::numbers::pi;
        auto flightTime =
            goalOffset.Norm() / (flywheelTangentialVelocity * units::math::cos(Constants::Flywheel::Angle));
        // TODO: account for turret rotation
        goalOffset = goalOffset - frc::Translation2d{dx * flightTime, dy * flightTime};
    }

    m_previousPose = pose;
    return frc::Rotation2d(units::math::atan2(goalOffset.Y(), goalOffset.X())) - pose.Rotation();
}