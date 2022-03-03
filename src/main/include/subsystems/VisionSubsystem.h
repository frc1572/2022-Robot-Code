#pragma once

#include <optional>

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimVisionSystem.h>
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
    std::optional<TargetInfo> PopLatestResult();
    void Periodic() override;
    void SimulationPeriodic() override;

private:
    photonlib::PhotonCamera m_camera{"limelight"};
    units::meter_t m_cameraHeight;
    units::meter_t m_targetHeight;
    units::degree_t m_cameraPitch;
    std::optional<TargetInfo> m_latestResult;

    frc::Pose2d m_simGoalPose{Constants::GoalTranslation, 0_deg};
    photonlib::SimVisionSystem m_cameraSim{
        "limelight", 77.6_deg, m_cameraPitch, {}, m_cameraHeight, 30_ft, 960, 720, 1};
    std::function<frc::Pose2d()> m_simPoseProvider;
};
