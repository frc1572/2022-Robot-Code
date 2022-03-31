#include "commands/PoseEstimatorCommand.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <spdlog/spdlog.h>
#include <units/angle.h>
#include <units/length.h>
#include <wpi/numbers>

#include "Constants.h"
#include "helper/AngleStatistics.h"

PoseEstimatorCommand::PoseEstimatorCommand(
    DriveTrainSubsystem& drivetrain, TurretSubsystem& turret, VisionSubsystem& vision)
  : m_drivetrain(drivetrain), m_turret(turret), m_vision(vision),
    m_observer(
        [](const Eigen::Vector<double, 4>& x, const Eigen::Vector<double, 4>& u) { return u; },
        [](const Eigen::Vector<double, 4>& x, const Eigen::Vector<double, 4>& u) { return x.block<2, 1>(2, 0); },
        {1.0, 1.0, 2.0, 4.0},
        {0.0000, 0.0000},
        AngleMean<4, 4>({2, 3}),
        AngleMean<2, 4>({0, 1}),
        AngleResidual<4>({2, 3}),
        AngleResidual<2>({0, 1}),
        AngleAdd<4>({2, 3}),
        Constants::LoopPeriod),
    m_visionMeasurementFn(
        [](const Eigen::Vector<double, 4>& x, const Eigen::Vector<double, 4>& u)
        {
            auto cameraRotation = x.block<2, 1>(2, 0).sum();
            Eigen::Vector<double, 2> cameraOffset{
                units::meter_t{Constants::CameraRotationRadius}.value() * std::cos(cameraRotation),
                units::meter_t{Constants::CameraRotationRadius}.value() * std::sin(cameraRotation)};
            auto cameraTranslation = x.block<2, 1>(0, 0) + cameraOffset;
            Eigen::Vector<double, 2> goalTranslation{
                Constants::GoalTranslation.X().value(), Constants::GoalTranslation.Y().value()};
            auto goalOffset = goalTranslation - cameraTranslation;
            double yaw = frc::InputModulus(
                atan2(goalOffset[1], goalOffset[0]) - cameraRotation, -wpi::numbers::pi, wpi::numbers::pi);
            double distance = goalOffset.norm() * std::cos(yaw);
            return Eigen::Vector<double, 2>{distance, yaw};
        }),
    m_visionCorrectionFn(
        [this](const Eigen::Vector<double, 4>& u, const Eigen::Vector<double, 2>& y)
        {
            m_observer.Correct<2>(
                u,
                y,
                m_visionMeasurementFn,
                frc::MakeCovMatrix<2>({1.0, 0.0}),
                AngleMean<2, 4>({1}),
                AngleResidual<2>({1}),
                AngleResidual<4>({2, 3}),
                AngleAdd<4>({2, 3}));
        })
{
    AddRequirements(&m_vision);
    SetName("PoseEstimatorCommand");
    Reset({}, {});
}

void PoseEstimatorCommand::Execute()
{
    auto previousTranslation = Eigen::Vector<double, 2>(m_observer.Xhat().block<2, 1>(0, 0));

    auto chassisSpeeds = m_drivetrain.GetMeasuredChassisSpeeds();
    auto drivetrainMeasuredAngle = m_drivetrain.GetMeasuredRotation();
    auto turretMeasuredAngle = m_turret.GetMeasuredRotation();

    auto fieldRelativeSpeeds =
        frc::Translation2d(chassisSpeeds.vx * 1_s, chassisSpeeds.vy * 1_s).RotateBy(drivetrainMeasuredAngle);
    Eigen::Vector<double, 4> u{
        fieldRelativeSpeeds.X().value(),
        fieldRelativeSpeeds.Y().value(),
        chassisSpeeds.omega.value(),
        m_turret.GetMeasuredVelocity().value()};
    Eigen::Vector<double, 2> localY{drivetrainMeasuredAngle.Radians().value(), turretMeasuredAngle.Radians().value()};

    m_latencyCompensator.AddObserverState(m_observer, u, localY, frc::Timer::GetFPGATimestamp());

    m_observer.Predict(u, Constants::LoopPeriod);
    m_observer.Correct(u, localY);

    if (auto targetInfo = m_vision.GetLatestResult())
    {
        // std::cout << "Popped Latest result! " << std::endl;
        auto expectedYaw = units::radian_t{m_visionMeasurementFn(m_observer.Xhat(), {})[1]};
        if (units::math::abs(targetInfo->yaw.Radians() - expectedYaw) < 45_deg)
        {
            Eigen::Vector<double, 2> visionMeasurement{targetInfo->distance.value(), targetInfo->yaw.Radians().value()};
            m_latencyCompensator.ApplyPastGlobalMeasurement<2>(
                &m_observer, Constants::LoopPeriod, visionMeasurement, m_visionCorrectionFn, targetInfo->timestamp);
        }
    }

    auto translationChange = m_observer.Xhat().block<2, 1>(0, 0) - previousTranslation;
    if (units::meter_t{translationChange.norm()} / Constants::LoopPeriod > Constants::SwerveModule::ThrottleMaxVelocity)
    {
        auto limitedTranslation = previousTranslation +
            translationChange.normalized() *
                units::meter_t{Constants::SwerveModule::ThrottleMaxVelocity * Constants::LoopPeriod}.value();
        m_observer.SetXhat(0, limitedTranslation[0]);
        m_observer.SetXhat(1, limitedTranslation[1]);
    }

    m_drivetrain.SetPose(GetPose());
    frc::SmartDashboard::PutNumber("CH X: ", chassisSpeeds.vx.value());
    frc::SmartDashboard::PutNumber("CH Y: ", chassisSpeeds.vy.value());
    frc::SmartDashboard::PutNumber("CH Omega: ", chassisSpeeds.omega.value());
    frc::SmartDashboard::PutNumber("FR X: ", fieldRelativeSpeeds.X().value());
    frc::SmartDashboard::PutNumber("FR Y: ", fieldRelativeSpeeds.Y().value());
}

bool PoseEstimatorCommand::RunsWhenDisabled() const
{
    return true;
}

frc::Pose2d PoseEstimatorCommand::GetPose()
{
    return {m_observer.Xhat(0) * 1_m, m_observer.Xhat(1) * 1_m, frc::Rotation2d(units::radian_t(m_observer.Xhat(2)))};
}

void PoseEstimatorCommand::Reset(frc::Pose2d currentPose, frc::Rotation2d currentTurretRotation)
{
    m_observer.SetXhat(
        {currentPose.X().value(),
         currentPose.Y().value(),
         currentPose.Rotation().Radians().value(),
         currentTurretRotation.Radians().value()});
}

Eigen::Vector<double, 2> CalculateVisionMeasurement(
    const Eigen::Vector<double, 4>& x, const Eigen::Vector<double, 4>& u)
{
    Eigen::Vector<double, 2> cameraOffset{
        units::meter_t{Constants::CameraRotationRadius}.value() * std::cos(x[3]),
        units::meter_t{Constants::CameraRotationRadius}.value() * std::sin(x[3])};
    auto cameraTranslation = x.block<2, 1>(0, 0) + cameraOffset;
    auto cameraRotation = x.block<2, 1>(2, 0).sum();
    Eigen::Vector<double, 2> goalTranslation{
        Constants::GoalTranslation.X().value(), Constants::GoalTranslation.Y().value()};
    auto goalOffset = goalTranslation - cameraTranslation;
    double distance = goalOffset.norm();
    double yaw =
        frc::InputModulus(atan2(goalOffset[1], goalOffset[0]) - cameraRotation, -wpi::numbers::pi, wpi::numbers::pi);
    return Eigen::Vector<double, 2>{distance, yaw};
}