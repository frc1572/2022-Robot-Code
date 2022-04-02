#pragma once

#include <functional>
#include <string>

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/SPI.h>
#include <frc/Timer.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <pathplanner/lib/PathPlanner.h>
#include <units/angle.h>
#include <wpi/array.h>

#include "Constants.h"
#include "SwerveModuleSubsystem.h"

class DriveTrainSubsystem : public frc2::SubsystemBase
{
public:
    DriveTrainSubsystem();
    void Drive(frc::ChassisSpeeds&& chassisSpeeds);
    frc::ChassisSpeeds GetDesiredChassisSpeeds();
    frc::ChassisSpeeds GetMeasuredChassisSpeeds();
    frc::ChassisSpeeds GetInvertedChassisSpeeds();
    frc::Rotation2d GetMeasuredRotation();
    frc::Pose2d GetPose();
    void TestDrive();
    void Periodic() override;
    void SimulationPeriodic() override;
    void Reset(frc::Pose2d currentPose);
    frc2::SwerveControllerCommand<4> MakeDrivePathPlannerCommand(
        /*std::string name,*/ pathplanner::PathPlannerTrajectory
            trajectory /*, std::function<void(frc::Pose2d)> resetPose*/);

private:
    WPI_Pigeon2 m_IMU{10, "canivore"};

    frc::ChassisSpeeds m_desiredChassisSpeeds;

    wpi::array<SwerveModuleSubsystem, 4> m_swerveModules{
        SwerveModuleSubsystem{1, 2, -1143 / Constants::TicksPerRevolution::TalonFX},
        SwerveModuleSubsystem{4, 3, -1415 / Constants::TicksPerRevolution::TalonFX},
        SwerveModuleSubsystem{5, 6, -410 / Constants::TicksPerRevolution::TalonFX},
        SwerveModuleSubsystem{8, 7, -1559 / Constants::TicksPerRevolution::TalonFX}};
    frc::SwerveDriveKinematics<4> m_swerveKinematics{
        frc::Translation2d{-11.75_in, 11.75_in},
        frc::Translation2d{11.75_in, 11.75_in},
        frc::Translation2d{11.75_in, -11.75_in},
        frc::Translation2d{-11.75_in, -11.75_in}};

    frc::PIDController m_translationController{4, 0.0, 0.0, Constants::LoopPeriod};

    frc::ProfiledPIDController<units::radians> m_headingController{
        4, 0.0, 0.0, {1440_deg_per_s, 1440_deg_per_s_sq}, Constants::LoopPeriod};

    frc::SwerveDriveOdometry<4> m_swerveOdometry{m_swerveKinematics, frc::Rotation2d(0_rad)};

    frc::Field2d m_field;
};