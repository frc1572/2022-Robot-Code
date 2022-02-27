#include "subsystems/VisionSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <photonlib/PhotonUtils.h>

VisionSubsystem::VisionSubsystem(units::meter_t cameraHeight, units::meter_t targetHeight, units::degree_t cameraPitch)
  : m_cameraHeight(cameraHeight), m_targetHeight(targetHeight), m_cameraPitch(cameraPitch)
{
    SetName("VisionSubsystem");
    m_camera.SetDriverMode(false);
    m_camera.SetLEDMode(photonlib::LEDMode::kOn);
    m_camera.SetPipelineIndex(0);
}

std::optional<VisionSubsystem::TargetInfo> VisionSubsystem::GetLatestResult()
{
    return m_latestResult;
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
        auto bestTarget = result.GetBestTarget();
        auto distance = photonlib::PhotonUtils::CalculateDistanceToTarget(
            m_cameraHeight, m_targetHeight, m_cameraPitch, units::degree_t(bestTarget.GetPitch()));
        auto yaw = frc::Rotation2d(units::degree_t(-bestTarget.GetYaw()));
        auto latency = result.GetLatency();
        m_latestResult = {distance, yaw, latency};
        frc::SmartDashboard::PutBoolean("VisionSubsystem.HasTarget", true);
        frc::SmartDashboard::PutNumber("VisionSubsystem.GoalDistanceMeters", distance.value());
        frc::SmartDashboard::PutNumber("VisionSubsystem.GoalYawDegrees", yaw.Degrees().value());
        frc::SmartDashboard::PutNumber("VisionSubsystem.LatencySeconds", latency.value());
    }
}