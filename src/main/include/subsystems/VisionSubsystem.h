#pragma once

#include <memory>
#include <optional>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <units/length.h>
#include <units/time.h>

#include "Constants.h"

class VisionSubsystem : public frc2::SubsystemBase
{
public:
    struct TargetInfo
    {
        units::meter_t distance;
        frc::Rotation2d yaw;
        units::second_t timestamp;
    };

    VisionSubsystem(
        units::meter_t cameraHeight,
        units::meter_t targetHeight,
        units::degree_t cameraPitch,
        std::function<frc::Pose2d()> poseProvider);
    std::optional<TargetInfo> GetLatestResult();
    void Periodic() override;

private:
    std::shared_ptr<nt::NetworkTable> m_table;
    units::meter_t m_cameraHeight;
    units::meter_t m_targetHeight;
    units::degree_t m_cameraPitch;
    std::optional<TargetInfo> m_latestResult;
};
