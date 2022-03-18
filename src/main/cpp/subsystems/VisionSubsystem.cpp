#include "subsystems/VisionSubsystem.h"

#include <functional>
#include <iostream>

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <photonlib/PhotonUtils.h>
#include <photonlib/SimVisionTarget.h>
#include <spdlog/spdlog.h>

VisionSubsystem::VisionSubsystem(
    units::meter_t cameraHeight,
    units::meter_t targetHeight,
    units::degree_t cameraPitch,
    std::function<frc::Pose2d()> simPoseProvider)
  : m_cameraHeight(cameraHeight), m_targetHeight(targetHeight), m_cameraPitch(cameraPitch),
    m_simPoseProvider(simPoseProvider)
{
    SetName("VisionSubsystem");
    m_camera.SetDriverMode(false);
    m_camera.SetLEDMode(photonlib::LEDMode::kOn);
    m_camera.SetPipelineIndex(0);
    m_cameraSim.AddSimVisionTarget(photonlib::SimVisionTarget{m_simGoalPose, m_targetHeight, 62.75_in, 12_in});
}

std::optional<VisionSubsystem::TargetInfo> VisionSubsystem::PopLatestResult()
{
    auto result = m_latestResult;
    m_latestResult = std::nullopt;
    return result;
}

void VisionSubsystem::Periodic()
{
    auto result = m_camera.GetLatestResult();

    if (!result.HasTargets())
    {
        m_latestResult = std::nullopt;
        frc::SmartDashboard::PutBoolean("VisionSubsystem.HasTarget", false);
    }
    else
    {
        // std::cout << "Hitting the Vision Else Branch! " << std::endl;
        auto bestTarget = result.GetBestTarget();
        auto distance = photonlib::PhotonUtils::CalculateDistanceToTarget(
                            m_cameraHeight, m_targetHeight, m_cameraPitch, units::degree_t(bestTarget.GetPitch())) +
            (frc::RobotBase::IsReal() ? Constants::UpperHubRadius : decltype(Constants::UpperHubRadius){0});
        auto yaw = frc::Rotation2d(units::degree_t(-bestTarget.GetYaw()));
        auto latency = result.GetLatency();
        m_latestResult = {distance, yaw, frc::Timer::GetFPGATimestamp() - latency};
        frc::SmartDashboard::PutBoolean("VisionSubsystem.HasTarget", true);
        frc::SmartDashboard::PutNumber("VisionSubsystem.GoalDistanceMeters", distance.value());
        frc::SmartDashboard::PutNumber("VisionSubsystem.GoalYawDegrees", yaw.Degrees().value());
        frc::SmartDashboard::PutNumber("VisionSubsystem.LatencySeconds", latency.value());
    }
}

void VisionSubsystem::SimulationPeriodic()
{
    auto pose = m_simPoseProvider();
    m_cameraSim.ProcessFrame(pose);
    // spdlog::info("{}, {}", pose.X().value(), pose.Y().value());
}
