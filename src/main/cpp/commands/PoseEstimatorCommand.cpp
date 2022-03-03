#include "commands/PoseEstimatorCommand.h"

#include <Eigen/Core>
#include <frc/estimator/AngleStatistics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/Timer.h>
#include <spdlog/spdlog.h>
#include <units/angle.h>
#include <units/length.h>
#include <wpi/numbers>

#include "Constants.h"

PoseEstimatorCommand::PoseEstimatorCommand(
    DriveTrainSubsystem& drivetrain, TurretSubsystem& turret, VisionSubsystem& vision)
  : m_drivetrain(drivetrain), m_turret(turret), m_vision(vision),
    m_observer(
        [](const Eigen::Vector<double, 3>& x, const Eigen::Vector<double, 3>& u) { return u; },
        [](const Eigen::Vector<double, 3>& x, const Eigen::Vector<double, 3>& u) { return x.block<1, 1>(2, 0); },
        {1.0, 1.0, 2.0},
        {0.0005},
        frc::AngleMean<3, 3>(2),
        frc::AngleMean<1, 3>(0),
        frc::AngleResidual<3>(2),
        frc::AngleResidual<1>(0),
        frc::AngleAdd<3>(2),
        Constants::LoopPeriod)
{
    AddRequirements(&m_vision);
    SetName("PoseEstimatorCommand");
}

void PoseEstimatorCommand::Execute()
{
    auto chassisSpeeds = m_drivetrain.GetMeasuredChassisSpeeds();
    auto measuredAngle = m_drivetrain.GetMeasuredRotation();

    auto fieldRelativeSpeeds =
        frc::Translation2d(chassisSpeeds.vx * 1_s, chassisSpeeds.vy * 1_s).RotateBy(measuredAngle);

    Eigen::Vector<double, 3> u{
        fieldRelativeSpeeds.X().value(), fieldRelativeSpeeds.Y().value(), chassisSpeeds.omega.value()};
    Eigen::Vector<double, 1> localY{measuredAngle.Radians().value()};

    m_latencyCompensator.AddObserverState(m_observer, u, localY, frc::Timer::GetFPGATimestamp());

    m_observer.Predict(u, Constants::LoopPeriod);
    m_observer.Correct(u, localY);

    if (auto targetInfo = m_vision.PopLatestResult())
    {
        Eigen::Vector<double, 2> visionMeasurement{targetInfo->distance.value(), targetInfo->yaw.Radians().value()};
        auto visionTimestamp = frc::Timer::GetFPGATimestamp() - targetInfo->latency;
        auto correctFn = [this](const Eigen::Vector<double, 3>& u, const Eigen::Vector<double, 2>& y)
        {
            m_observer.Correct<2>(
                u,
                y,
                [this](const Eigen::Vector<double, 3>& x, const Eigen::Vector<double, 3>& u)
                {
                    Eigen::Vector<double, 2> goalTranslation{
                        Constants::GoalTranslation.X().value(), Constants::GoalTranslation.Y().value()};
                    auto goalOffset = goalTranslation - x.block<2, 1>(0, 0);
                    double distance = goalOffset.norm();
                    double yaw = frc::InputModulus(
                        atan2(goalOffset[1], goalOffset[0]) - m_turret.GetMeasuredPosition().Radians().value() -
                            m_drivetrain.GetMeasuredRotation().Radians().value(),
                        -wpi::numbers::pi,
                        wpi::numbers::pi);
                    return Eigen::Vector<double, 2>{distance, yaw};
                },
                frc::MakeCovMatrix<2>({0.25, 0.02}),
                frc::AngleMean<2, 3>(1),
                frc::AngleResidual<2>(1),
                frc::AngleResidual<3>(2),
                frc::AngleAdd<3>(2));
        };
        m_latencyCompensator.ApplyPastGlobalMeasurement<2>(
            &m_observer, Constants::LoopPeriod, visionMeasurement, correctFn, visionTimestamp);
    }

    auto pose = frc::Pose2d(
        m_observer.Xhat(0) * 1_m, m_observer.Xhat(1) * 1_m, frc::Rotation2d(units::radian_t(m_observer.Xhat(2))));

    m_drivetrain.SetPose(pose);
}

bool PoseEstimatorCommand::RunsWhenDisabled() const
{
    return true;
}