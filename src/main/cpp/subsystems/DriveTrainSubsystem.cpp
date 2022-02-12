#include "subsystems/DriveTrainSubsystem.h"

#include <cmath>
#include <iostream>

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <wpi/numbers>

DriveTrainSubsystem::DriveTrainSubsystem()
{
    m_NavX.Reset();
    m_headingController.EnableContinuousInput(0, 360);
}

void DriveTrainSubsystem::Drive(frc::ChassisSpeeds&& chassisSpeeds)
{
    /*
    m_desiredHeading += chassisSpeeds.omega * Constants::LoopPeriod;
    auto headingOutput =
        m_headingController.Calculate(GetRotation().Degrees().value(), m_desiredHeading.value()) * 1_deg_per_s;
    if (units::math::abs(headingOutput) < 5_deg_per_s)
    {
        headingOutput = 0_deg_per_s;
    }
    chassisSpeeds.omega = headingOutput;
    */

    auto moduleStates = DriveTrainSubsystem::m_swerveKinematics.ToSwerveModuleStates(chassisSpeeds);
    m_swerveModules[0].SetDesiredState(moduleStates[0]);
    m_swerveModules[1].SetDesiredState(moduleStates[1]);
    m_swerveModules[2].SetDesiredState(moduleStates[2]);
    m_swerveModules[3].SetDesiredState(moduleStates[3]);

    frc::SmartDashboard::PutNumber("DriveTrain.MeasuredHeading", GetRotation().Degrees().value());
    frc::SmartDashboard::PutNumber("DriveTrain.DesiredHeading", m_desiredHeading.value());
    frc::SmartDashboard::PutNumber("DriveTrain.HeadingError", m_headingController.GetPositionError());
}

frc::Rotation2d DriveTrainSubsystem::GetRotation()
{
    frc::SmartDashboard::PutNumber("Gyro", m_NavX.GetAngle());
    return {m_NavX.GetAngle() * 1_deg};
}

void DriveTrainSubsystem::TestDrive()
{
    m_swerveModules[0].TestingVoltage();
    m_swerveModules[1].TestingVoltage();
    m_swerveModules[2].TestingVoltage();
    m_swerveModules[3].TestingVoltage();
}
// void DriveTrainSubsystem::Periodic() {
//     m_field.SetRobotPose(GetPose());
//     frc::SmartDashboard::PutData("Field", &m_field);
// }