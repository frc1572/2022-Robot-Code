#include "commands/DriveTeleopCommand.h"

DriveTeleopCommand::DriveTeleopCommand(
    DriveTrainSubsystem& drivetrain, frc::Joystick& TranslationJoystick, frc::Joystick& SteeringJoystick)
  : m_drivetrain(drivetrain), m_TranslationJoystick(TranslationJoystick), m_SteeringJoystick(SteeringJoystick)
{
    AddRequirements(&m_drivetrain);
}
void DriveTeleopCommand::Execute()
{
    m_drivetrain.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        -m_TranslationJoystick.GetY() * 10_mps,
        -m_TranslationJoystick.GetX() * 10_mps,
        m_SteeringJoystick.GetX() * 1_rad_per_s,
        m_drivetrain.GetRotation()));
}