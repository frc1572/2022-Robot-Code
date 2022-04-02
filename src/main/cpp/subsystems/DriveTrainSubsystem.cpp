#include "subsystems/DriveTrainSubsystem.h"

#include <iostream>
#include <vector>

#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <units/angular_velocity.h>
#include <wpi/numbers>

DriveTrainSubsystem::DriveTrainSubsystem()
{
    m_IMU.Reset();
    m_IMU.ConfigFactoryDefault();
    m_headingController.EnableContinuousInput(-180_deg, 180_deg);
    // m_IMU.SetYaw(0);
    //  Subsytem move constructor does not register, so we must do it by hand

    for (auto& module : m_swerveModules)
    {
        module.Register();
    }
}

void DriveTrainSubsystem::Drive(frc::ChassisSpeeds&& chassisSpeeds)
{
    m_desiredChassisSpeeds = chassisSpeeds;
    auto moduleStates = DriveTrainSubsystem::m_swerveKinematics.ToSwerveModuleStates(chassisSpeeds);
    for (unsigned int i = 0; i < m_swerveModules.size(); i++)
    {
        m_swerveModules[i].SetDesiredState(moduleStates[i]);
    }
}

frc::ChassisSpeeds DriveTrainSubsystem::GetDesiredChassisSpeeds()
{
    return m_desiredChassisSpeeds;
}

frc::ChassisSpeeds DriveTrainSubsystem::GetMeasuredChassisSpeeds()
{
    return m_swerveKinematics.ToChassisSpeeds(
        m_swerveModules[0].GetMeasuredState(),
        m_swerveModules[1].GetMeasuredState(),
        m_swerveModules[2].GetMeasuredState(),
        m_swerveModules[3].GetMeasuredState());
}

frc::ChassisSpeeds DriveTrainSubsystem::GetInvertedChassisSpeeds()
{
}

frc::Rotation2d DriveTrainSubsystem::GetMeasuredRotation()
{
    return GetPose().Rotation();
}

frc::Pose2d DriveTrainSubsystem::GetPose()
{
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
    m_swerveOdometry.UpdateWithTime(
        frc::Timer::GetFPGATimestamp(),
        m_IMU.GetRotation2d(),
        m_swerveModules[0].GetMeasuredState(),
        m_swerveModules[1].GetMeasuredState(),
        m_swerveModules[2].GetMeasuredState(),
        m_swerveModules[3].GetMeasuredState());
    auto pose = GetPose();
    m_field.SetRobotPose(pose);
    frc::SmartDashboard::PutData("Field", &m_field);
    // frc::SmartDashboard::PutNumber("DriveTrain.MeasuredIMUYaw", m_IMU.GetRotation2d().Degrees().value());
}

void DriveTrainSubsystem::SimulationPeriodic()
{
    auto chassisSpeeds = GetMeasuredChassisSpeeds();
    auto imuSim = m_IMU.GetSimCollection();
    imuSim.AddHeading(units::degree_t(chassisSpeeds.omega * Constants::LoopPeriod).value());
}

void DriveTrainSubsystem::Reset(frc::Pose2d currentPose)
{
    m_swerveOdometry.ResetPosition(currentPose, m_IMU.GetRotation2d());
    for (auto& module : m_swerveModules)
    {
        module.Reset();
    }
}

frc2::SwerveControllerCommand<4> DriveTrainSubsystem::MakeDrivePathPlannerCommand(
    /*std::string name,*/ pathplanner::PathPlannerTrajectory
        trajectory /*, std::function<void(frc::Pose2d)> resetPose*/)
{
    auto initialState = *trajectory.getState(0);

    std::vector<frc::Trajectory::State> wpilibStates(trajectory.numStates());
    for (auto& state : *trajectory.getStates())
    {
        wpilibStates.push_back({state.time, state.velocity, state.acceleration, state.pose, state.curvature});
    }

    auto cmd = frc2::SwerveControllerCommand<4>(
        frc::Trajectory(wpilibStates),
        [this]() { return GetPose(); },
        m_swerveKinematics,
        m_translationController,
        m_translationController,
        m_headingController,
        [trajectory, timer = frc::Timer()]() mutable
        {
            if (timer.HasElapsed(trajectory.getTotalTime()))
            {
                timer.Reset();
            }
            timer.Start();
            return trajectory.sample(timer.Get()).holonomicRotation;
        },
        [this](std::array<frc::SwerveModuleState, 4> moduleStates)
        {
            for (unsigned int i = 0; i < m_swerveModules.size(); i++)
            {
                m_swerveModules[i].SetDesiredState(moduleStates[i]);
            }
        },
        {this})
        /*
        [initialState, resetPose]() {
            resetPose({initialState.pose.Translation(), initialState.holonomicRotation});
        })*/
        ;
    std::cout << "X: " << initialState.pose.Translation().X().value()
              << "Y: " << initialState.pose.Translation().Y().value()
              << "Initial Rotation: " << initialState.holonomicRotation.Radians().value() << std::endl;
    // cmd.SetName(name);
    return cmd;
}