#include "subsystems/DriveTrainSubsystem.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <units/angular_velocity.h>
#include <wpi/numbers>

DriveTrainSubsystem::DriveTrainSubsystem()
{
    m_IMU.Reset();
    m_headingController.EnableContinuousInput(0, 360);
    // Subsytem move constructor does not register, so we must do it by hand
    for (auto& module : m_swerveModules)
    {
        module.Register();
    }
}

void DriveTrainSubsystem::Drive(frc::ChassisSpeeds&& chassisSpeeds)
{
    m_desiredHeading += chassisSpeeds.omega * Constants::LoopPeriod;
    auto headingOutput =
        m_headingController.Calculate(GetRotation().Degrees().value(), m_desiredHeading.value()) * 1_deg_per_s;
    if (units::math::abs(headingOutput) < 5_deg_per_s)
    {
        headingOutput = 0_deg_per_s;
    }
    chassisSpeeds.omega = headingOutput;

    auto moduleStates = DriveTrainSubsystem::m_swerveKinematics.ToSwerveModuleStates(chassisSpeeds);
    for (unsigned int i = 0; i < m_swerveModules.size(); i++)
    {
        m_swerveModules[i].SetDesiredState(moduleStates[i]);
    }

    frc::SmartDashboard::PutNumber("DriveTrain.MeasuredHeading", GetRotation().Degrees().value());
    frc::SmartDashboard::PutNumber("DriveTrain.DesiredHeading", m_desiredHeading.value());
    frc::SmartDashboard::PutNumber("DriveTrain.HeadingError", m_headingController.GetPositionError());
}

frc::Rotation2d DriveTrainSubsystem::GetRotation()
{
    return m_IMU.GetRotation2d();
}

frc::Pose2d DriveTrainSubsystem::GetPose()
{
    // TODO: use a kalman filter to estimate pose instead of odometry
    m_swerveOdometry.UpdateWithTime(
        frc::Timer::GetFPGATimestamp(),
        GetRotation(),
        m_swerveModules[0].GetMeasuredState(),
        m_swerveModules[1].GetMeasuredState(),
        m_swerveModules[2].GetMeasuredState(),
        m_swerveModules[3].GetMeasuredState());
    return m_swerveOdometry.GetPose();
}

void DriveTrainSubsystem::TestDrive()
{
    for (auto& module : m_swerveModules)
    {
        module.TestingVoltage();
    }
}
void DriveTrainSubsystem::Periodic()
{
    m_field.SetRobotPose(GetPose());
    frc::SmartDashboard::PutData("Field", &m_field);
    frc::SmartDashboard::PutNumber("Gyro", m_IMU.GetRotation2d().Degrees().value());
}

void DriveTrainSubsystem::SimulationPeriodic()
{
    auto chassisSpeeds = m_swerveKinematics.ToChassisSpeeds(
        m_swerveModules[0].GetMeasuredState(),
        m_swerveModules[1].GetMeasuredState(),
        m_swerveModules[2].GetMeasuredState(),
        m_swerveModules[3].GetMeasuredState());

    auto imuSim = m_IMU.GetSimCollection();
    imuSim.AddHeading(units::degree_t(chassisSpeeds.omega * Constants::LoopPeriod).value());
}

void DriveTrainSubsystem::Reset()
{
    m_IMU.Reset();
    for (auto& module : m_swerveModules)
    {
        module.Reset();
    }
}