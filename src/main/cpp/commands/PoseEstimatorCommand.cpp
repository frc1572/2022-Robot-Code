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
        [](const Eigen::Vector<double, 4>& x, const Eigen::Vector<double, 4>& u) { return u; },
        [](const Eigen::Vector<double, 4>& x, const Eigen::Vector<double, 4>& u) { return x.block<2, 1>(2, 0); },
        {1.0, 1.0, 2.0, 2.0},
        {0.0005, 0.0005},
        [](const Eigen::Matrix<double, 4, 2 * 4 + 1>& sigmas, const Eigen::Vector<double, 2 * 4 + 1>& Wm)
        {
            Eigen::Vector<double, 4> ret = sigmas * Wm;
            for (int idx : {2, 3})
            {
                double sumSin = sigmas.row(idx).unaryExpr([](auto it) { return std::sin(it); }).sum();
                double sumCos = sigmas.row(idx).unaryExpr([](auto it) { return std::cos(it); }).sum();
                ret[idx] = std::atan2(sumSin, sumCos);
            }
            return ret;
        },
        [](const Eigen::Matrix<double, 2, 2 * 4 + 1>& sigmas, const Eigen::Vector<double, 2 * 4 + 1>& Wm)
        {
            Eigen::Vector<double, 2> ret = sigmas * Wm;
            for (int idx : {0, 1})
            {
                double sumSin = sigmas.row(idx).unaryExpr([](auto it) { return std::sin(it); }).sum();
                double sumCos = sigmas.row(idx).unaryExpr([](auto it) { return std::cos(it); }).sum();
                ret[idx] = std::atan2(sumSin, sumCos);
            }
            return ret;
        },
        [](const Eigen::Vector<double, 4>& a, const Eigen::Vector<double, 4>& b)
        {
            Eigen::Vector<double, 4> ret = a - b;
            for (int idx : {2, 3})
            {
                ret[idx] = frc::AngleModulus(units::radian_t{ret[idx]}).value();
            }
            return ret;
        },
        [](const Eigen::Vector<double, 2>& a, const Eigen::Vector<double, 2>& b)
        {
            Eigen::Vector<double, 2> ret = a - b;
            for (int idx : {0, 1})
            {
                ret[idx] = frc::AngleModulus(units::radian_t{ret[idx]}).value();
            }
            return ret;
        },
        [](const Eigen::Vector<double, 4>& a, const Eigen::Vector<double, 4>& b)
        {
            Eigen::Vector<double, 4> ret = a + b;
            for (int idx : {2, 3})
            {
                {
                    ret[idx] = frc::InputModulus(ret[idx], -wpi::numbers::pi, wpi::numbers::pi);
                }
            }
            return ret;
        },
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

    Eigen::Vector<double, 4> u{
        fieldRelativeSpeeds.X().value(),
        fieldRelativeSpeeds.Y().value(),
        chassisSpeeds.omega.value(),
        m_turret.GetMeasuredVelocity().value()};
    Eigen::Vector<double, 2> localY{measuredAngle.Radians().value(), m_turret.GetMeasuredPosition().Radians().value()};

    m_latencyCompensator.AddObserverState(m_observer, u, localY, frc::Timer::GetFPGATimestamp());

    m_observer.Predict(u, Constants::LoopPeriod);
    m_observer.Correct(u, localY);

    if (auto targetInfo = m_vision.PopLatestResult())
    {
        Eigen::Vector<double, 2> visionMeasurement{targetInfo->distance.value(), targetInfo->yaw.Radians().value()};
        auto visionTimestamp = frc::Timer::GetFPGATimestamp() - targetInfo->latency;
        auto correctFn = [this](const Eigen::Vector<double, 4>& u, const Eigen::Vector<double, 2>& y)
        {
            m_observer.Correct<2>(
                u,
                y,
                [](const Eigen::Vector<double, 4>& x, const Eigen::Vector<double, 4>& u)
                {
                    Eigen::Vector<double, 2> goalTranslation{
                        Constants::GoalTranslation.X().value(), Constants::GoalTranslation.Y().value()};
                    auto goalOffset = goalTranslation - x.block<2, 1>(0, 0);
                    double distance = goalOffset.norm();
                    double yaw = frc::InputModulus(
                        atan2(goalOffset[1], goalOffset[0]) - x[3] - x[2], -wpi::numbers::pi, wpi::numbers::pi);
                    return Eigen::Vector<double, 2>{distance, yaw};
                },
                frc::MakeCovMatrix<2>({0.25, 0.02}),
                [](const Eigen::Matrix<double, 2, 2 * 4 + 1>& sigmas, const Eigen::Vector<double, 2 * 4 + 1>& Wm)
                {
                    Eigen::Vector<double, 2> ret = sigmas * Wm;
                    for (int idx : {0, 1})
                    {
                        double sumSin = sigmas.row(idx).unaryExpr([](auto it) { return std::sin(it); }).sum();
                        double sumCos = sigmas.row(idx).unaryExpr([](auto it) { return std::cos(it); }).sum();
                        ret[idx] = std::atan2(sumSin, sumCos);
                    }
                    return ret;
                },
                [](const Eigen::Vector<double, 2>& a, const Eigen::Vector<double, 2>& b)
                {
                    Eigen::Vector<double, 2> ret = a - b;
                    for (int idx : {0, 1})
                    {
                        ret[idx] = frc::AngleModulus(units::radian_t{ret[idx]}).value();
                    }
                    return ret;
                },
                [](const Eigen::Vector<double, 4>& a, const Eigen::Vector<double, 4>& b)
                {
                    Eigen::Vector<double, 4> ret = a - b;
                    for (int idx : {2, 3})
                    {
                        ret[idx] = frc::AngleModulus(units::radian_t{ret[idx]}).value();
                    }
                    return ret;
                },
                [](const Eigen::Vector<double, 4>& a, const Eigen::Vector<double, 4>& b)
                {
                    Eigen::Vector<double, 4> ret = a + b;
                    for (int idx : {2, 3})
                    {
                        {
                            ret[idx] = frc::InputModulus(ret[idx], -wpi::numbers::pi, wpi::numbers::pi);
                        }
                    }
                    return ret;
                });
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