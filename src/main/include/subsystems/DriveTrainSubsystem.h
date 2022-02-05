#pragma once
#include <AHRS.h>

#include <ctre/Phoenix.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/SPI.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <wpi/array.h>

#include "SwerveModuleSubsystem.h"

class DriveTrainSubsystem : public frc2::SubsystemBase
{
public:
    void Drive(frc::ChassisSpeeds&& chassisSpeeds);
    frc::Rotation2d GetRotation();
    // void Periodic() override;

private:
    AHRS m_NavX{frc::SPI::Port::kMXP};

    wpi::array<SwerveModuleSubsystem, 4> m_swerveModules{
        SwerveModuleSubsystem{1, 2, 0_deg},
        SwerveModuleSubsystem{4, 3, 0_deg},
        SwerveModuleSubsystem{5, 6, 0_deg},
        SwerveModuleSubsystem{8, 7, 0_deg}};
    frc::SwerveDriveKinematics<4> m_swerveKinematics{
        frc::Translation2d{11.75_in, 11.75_in},
        frc::Translation2d{-11.75_in, 11.75_in},
        frc::Translation2d{-11.75_in, -11.75_in},
        frc::Translation2d{11.75_in, -11.75_in}};
};