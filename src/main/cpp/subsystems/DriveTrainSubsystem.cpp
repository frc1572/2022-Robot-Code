#include "subsystems/DriveTrainSubsystem.h"

void DriveTrainSubsystem::Drive(frc::ChassisSpeeds&& chassisSpeeds)
{
    auto moduleStates = DriveTrainSubsystem::m_swerveKinematics.ToSwerveModuleStates(chassisSpeeds);
    m_swerveModules[0].SetDesiredState(moduleStates[0]);
    m_swerveModules[1].SetDesiredState(moduleStates[1]);
    m_swerveModules[2].SetDesiredState(moduleStates[2]);
    m_swerveModules[3].SetDesiredState(moduleStates[3]);
}

frc::Rotation2d DriveTrainSubsystem::GetRotation()
{
    return {m_NavX.GetAngle() * 1_deg};
}

// void DriveTrainSubsystem::Periodic() {
//     m_field.SetRobotPose(GetPose());
//     frc::SmartDashboard::PutData("Field", &m_field);
// }