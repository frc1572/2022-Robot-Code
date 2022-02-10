#pragma once
#include <AHRS.h>

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
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
    void TestDrive();
    // void Periodic() override;

private:
    AHRS m_NavX{frc::SPI::Port::kMXP};

    wpi::array<SwerveModuleSubsystem, 4> m_swerveModules{
        SwerveModuleSubsystem{1, 2, 0, -0.5603851_tr},
        SwerveModuleSubsystem{4, 3, 3, -0.442999_tr},
        SwerveModuleSubsystem{5, 6, 2, 0.0322221_tr},
        SwerveModuleSubsystem{8, 7, 1, -0.577164_tr}};
    frc::SwerveDriveKinematics<4> m_swerveKinematics{
        frc::Translation2d{-11.75_in, -11.75_in},
        frc::Translation2d{11.75_in, -11.75_in},
        frc::Translation2d{11.75_in, 11.75_in},
        frc::Translation2d{-11.75_in, 11.75_in}};

    units::degree_t m_desiredHeading;
    frc::PIDController m_headingController{4, 0.0, 0.0, Constants::LoopPeriod};
};