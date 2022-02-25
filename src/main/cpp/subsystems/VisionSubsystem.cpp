#include "subsystems/VisionSubsystem.h"

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
    }
    else
    {
        auto bestTarget = result.GetBestTarget();
        m_latestResult = {
            photonlib::PhotonUtils::CalculateDistanceToTarget(
                m_cameraHeight, m_targetHeight, m_cameraPitch, units::degree_t(bestTarget.GetPitch())),
            frc::Rotation2d(units::degree_t(-bestTarget.GetYaw())),
            result.GetLatency(),
        };
    }
}