#pragma once

#include <array>
#include <functional>
#include <initializer_list>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/Timer.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Subsystem.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <pathplanner/lib/PathPlanner.h>
#include <units/angle.h>

class DrivePathPlannerCommand : public frc2::CommandHelper<frc2::CommandBase, DrivePathPlannerCommand>
{
public:
    DrivePathPlannerCommand(
        pathplanner::PathPlannerTrajectory trajectory,
        std::function<frc::Pose2d()> pose,
        frc::SwerveDriveKinematics<4> kinematics,
        frc::PIDController xController,
        frc::PIDController yController,
        frc::ProfiledPIDController<units::radians> thetaController,
        std::function<void(std::array<frc::SwerveModuleState, 4>)> output,
        std::initializer_list<frc2::Subsystem*> requirements);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

    static frc::Trajectory PathPlannerToWPILibTrajectory(pathplanner::PathPlannerTrajectory trajectory);

private:
    pathplanner::PathPlannerTrajectory m_trajectory;
    frc2::SwerveControllerCommand<4> m_swerveControllerCommand;
    frc::Timer m_timer;
};