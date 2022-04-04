#include "subsystems/VisionSubsystem.h"

#include <algorithm>
#include <functional>
#include <iostream>

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <networktables/NetworkTableInstance.h>
#include <spdlog/spdlog.h>

VisionSubsystem::VisionSubsystem(
    units::meter_t cameraHeight,
    units::meter_t targetHeight,
    units::degree_t cameraPitch,
    std::function<frc::Pose2d()> simPoseProvider)
  : m_table(nt::NetworkTableInstance::GetDefault().GetTable("limelight")), m_cameraHeight(cameraHeight),
    m_targetHeight(targetHeight), m_cameraPitch(cameraPitch)
{
    SetName("VisionSubsystem");
    m_table->PutNumber("ledMode", 3); // force on
    m_table->PutNumber("camMode", 0); // processing mode
    m_table->PutNumber("pipeline", 0);
}

std::optional<VisionSubsystem::TargetInfo> VisionSubsystem::GetLatestResult()
{
    return m_latestResult;
}

void VisionSubsystem::Periodic()
{
    if (m_table->GetNumber("tv", 0) == 0)
    {
        m_latestResult = std::nullopt;
        frc::SmartDashboard::PutBoolean("VisionSubsystem.HasTarget", false);
    }

    else
    {
        auto tx = units::degree_t{-m_table->GetNumber("tx", 0)};
        auto ty = units::degree_t{m_table->GetNumber("ty", 0)};
        auto tl = units::millisecond_t{m_table->GetNumber("tl", 0)} + 11_ms;

        auto distance = m_filter.Calculate(
            (m_targetHeight - m_cameraHeight) / units::math::tan(m_cameraPitch + ty) + Constants::UpperHubRadius);
        m_latestResult = {distance, tx, frc::Timer::GetFPGATimestamp() - tl};
        frc::SmartDashboard::PutBoolean("VisionSubsystem.HasTarget", true);
        frc::SmartDashboard::PutNumber("VisionSubsystem.GoalDistanceMeters", distance.value());
        frc::SmartDashboard::PutNumber("VisionSubsystem.GoalYawDegrees", tx.value());
        frc::SmartDashboard::PutNumber("VisionSubsystem.LatencySeconds", tl.value());
    }
}
