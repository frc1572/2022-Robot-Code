#pragma once

#include <frc/estimator/KalmanFilterLatencyCompensator.h>
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class PoseEstimatorCommand : public frc2::CommandHelper<frc2::CommandBase, PoseEstimatorCommand>
{
public:
    PoseEstimatorCommand(DriveTrainSubsystem& drivetrain, TurretSubsystem& turret, VisionSubsystem& vision);
    void Execute() override;
    bool RunsWhenDisabled() const override;

private:
    DriveTrainSubsystem& m_drivetrain;
    TurretSubsystem& m_turret;
    VisionSubsystem& m_vision;

    frc::UnscentedKalmanFilter<4, 4, 2> m_observer;
    frc::KalmanFilterLatencyCompensator<4, 4, 2, frc::UnscentedKalmanFilter<4, 4, 2>> m_latencyCompensator;
};