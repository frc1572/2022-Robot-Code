#pragma once

#include <optional>

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <units/length.h>
#include <units/time.h>

class VisionSubsystem : public frc2::SubsystemBase
{
public:
    struct TargetInfo
    {
        units::meter_t distance;
        frc::Rotation2d yaw;
        units::second_t latency;
    };

    VisionSubsystem(units::meter_t cameraHeight, units::meter_t targetHeight, units::degree_t cameraPitch);
    std::optional<TargetInfo> PopLatestResult();
    void Periodic() override;

private:
    photonlib::PhotonCamera m_camera{"limelight"};
    units::meter_t m_cameraHeight;
    units::meter_t m_targetHeight;
    units::degree_t m_cameraPitch;
    std::optional<TargetInfo> m_latestResult;
};
