#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/SPI.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <wpi/array.h>

#include "Constants.h"
#include "SwerveModuleSubsystem.h"

class DriveTrainSubsystem : public frc2::SubsystemBase
{
public:
    DriveTrainSubsystem();
    void Drive(frc::ChassisSpeeds&& chassisSpeeds);
    frc::Rotation2d GetRotation();
    frc::Pose2d GetPose();
    void TestDrive();
    void Periodic() override;
    void SimulationPeriodic() override;
    void Reset();

private:
    WPI_Pigeon2 m_IMU{10, "canivore"};

    wpi::array<SwerveModuleSubsystem, 4> m_swerveModules{
        SwerveModuleSubsystem{1, 2, 1186 / Constants::TicksPerRevolution::TalonFX},
        SwerveModuleSubsystem{4, 3, 848.887 / Constants::TicksPerRevolution::TalonFX},
        SwerveModuleSubsystem{5, 6, -564.446 / Constants::TicksPerRevolution::TalonFX},
        SwerveModuleSubsystem{8, 7, 894.198 / Constants::TicksPerRevolution::TalonFX}};
    frc::SwerveDriveKinematics<4> m_swerveKinematics{
        frc::Translation2d{-11.75_in, -11.75_in},
        frc::Translation2d{11.75_in, -11.75_in},
        frc::Translation2d{11.75_in, 11.75_in},
        frc::Translation2d{-11.75_in, 11.75_in}};

    units::degree_t m_desiredHeading;
    frc::PIDController m_headingController{4, 0.0, 0.0, Constants::LoopPeriod};

    frc::SwerveDriveOdometry<4> m_swerveOdometry{m_swerveKinematics, frc::Rotation2d(0_rad)};

    frc::Field2d m_field;
};