#pragma once

#include <functional>

#include <Eigen/Core>
#include <frc/estimator/KalmanFilterLatencyCompensator.h>
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>

#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class PoseEstimatorCommand : public frc2::CommandHelper<frc2::CommandBase, PoseEstimatorCommand>
{
public:
    PoseEstimatorCommand(DriveTrainSubsystem& drivetrain, TurretSubsystem& turret, VisionSubsystem& vision);
    void Execute() override;
    bool RunsWhenDisabled() const override;
    frc::Pose2d GetPose();
    void Reset(frc::Pose2d currentPose, frc::Rotation2d currentTurretRotation);

private:
    DriveTrainSubsystem& m_drivetrain;
    TurretSubsystem& m_turret;
    VisionSubsystem& m_vision;

    frc::Rotation2d m_drivetrainRotationOffset;
    frc::Rotation2d m_turretRotationOffset;

    frc::UnscentedKalmanFilter<4, 4, 2> m_observer;
    frc::KalmanFilterLatencyCompensator<4, 4, 2, frc::UnscentedKalmanFilter<4, 4, 2>> m_latencyCompensator;

    std::function<Eigen::Vector<double, 2>(const Eigen::Vector<double, 4>& x, const Eigen::Vector<double, 4>& u)>
        m_visionMeasurementFn;
    std::function<void(const Eigen::Vector<double, 4>& u, const Eigen::Vector<double, 2>& y)> m_visionCorrectionFn;
};