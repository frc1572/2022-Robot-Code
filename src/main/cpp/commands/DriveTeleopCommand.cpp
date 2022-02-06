#include "commands/DriveTeleopCommand.h"

DriveTeleopCommand::DriveTeleopCommand(
    DriveTrainSubsystem& drivetrain, frc::Joystick& translationJoystick, frc::Joystick& steeringJoystick)
  : m_drivetrain(drivetrain), m_translationJoystick(translationJoystick), m_steeringJoystick(steeringJoystick)
{
    AddRequirements(&m_drivetrain);
}
void DriveTeleopCommand::Execute()
{
    // m_drivetrain.TestDrive();
    m_drivetrain.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        -m_translationJoystick.GetY() * Constants::SwerveModule::ThrottleMaxVelocity,
        -m_translationJoystick.GetX() * Constants::SwerveModule::ThrottleMaxVelocity,
        m_steeringJoystick.GetX() * 1_rad_per_s,
        m_drivetrain.GetRotation()));
}