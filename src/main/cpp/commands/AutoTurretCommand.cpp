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
    // m_previousDesiredAngle = CalculateDesiredAngle();
    m_previousPose = m_drivetrain.GetPose();
}

void AutoTurretCommand::Execute()
{
    if (auto turretAngle = m_vision.GetLatestResult())
    {
        auto desiredAngle = m_turret.GetMeasuredRotation() + turretAngle->yaw;
        m_turret.SetDesiredPosition(desiredAngle, rad_per_s_t{0});
    }
    else
    {
        auto pose = m_drivetrain.GetPose();
        auto goalOffset = Constants::GoalTranslation - pose.Translation();
        auto desiredAngle = frc::Rotation2d{units::math::atan2(goalOffset.Y(), goalOffset.X())} - pose.Rotation();
        m_turret.SetDesiredPosition(desiredAngle, rad_per_s_t{0});
    }
    // m_previousDesiredAngle = desiredAngle;
}

frc::Rotation2d AutoTurretCommand::CalculateInertiaCompensationAngle()
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
        return {units::math::atan2(dy * flightTime, dx * flightTime)};
    }
    else
    {
        return {0_deg};
    }
}