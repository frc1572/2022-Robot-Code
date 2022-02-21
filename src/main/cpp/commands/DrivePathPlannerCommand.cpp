#include "commands/DrivePathPlannerCommand.h"

#include <vector>

DrivePathPlannerCommand::DrivePathPlannerCommand(
    pathplanner::PathPlannerTrajectory trajectory,
    std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<4> kinematics,
    frc::PIDController xController,
    frc::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<void(std::array<frc::SwerveModuleState, 4>)> output,
    std::initializer_list<frc2::Subsystem*> requirements)
  : m_trajectory(trajectory), m_swerveControllerCommand(
                                  PathPlannerToWPILibTrajectory(m_trajectory),
                                  pose,
                                  kinematics,
                                  xController,
                                  yController,
                                  thetaController,
                                  [this]() { return m_trajectory.sample(m_timer.Get()).holonomicRotation; },
                                  output,
                                  requirements)
{
}

void DrivePathPlannerCommand::Initialize()
{
    m_timer.Reset();
    m_timer.Start();
    m_swerveControllerCommand.Initialize();
}

void DrivePathPlannerCommand::Execute()
{
    m_swerveControllerCommand.Execute();
}

bool DrivePathPlannerCommand::IsFinished()
{
    return m_swerveControllerCommand.IsFinished();
}

void DrivePathPlannerCommand::End(bool interrupted)
{
    m_swerveControllerCommand.End(interrupted);
    m_timer.Stop();
}

frc::Trajectory DrivePathPlannerCommand::PathPlannerToWPILibTrajectory(pathplanner::PathPlannerTrajectory trajectory)
{
    std::vector<frc::Trajectory::State> states(trajectory.numStates());
    for (auto& state : *trajectory.getStates())
    {
        states.push_back({state.time, state.velocity, state.acceleration, state.pose, state.curvature});
    }
    return frc::Trajectory(states);
}