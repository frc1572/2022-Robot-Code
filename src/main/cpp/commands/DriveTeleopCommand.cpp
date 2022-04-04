#include "commands/DriveTeleopCommand.h"

#include <cmath>
#include <iostream>

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

DriveTeleopCommand::DriveTeleopCommand(
    DriveTrainSubsystem& drivetrain, frc::Joystick& translationJoystick, frc::Joystick& steeringJoystick)
  : m_drivetrain(drivetrain), m_translationJoystick(translationJoystick), m_steeringJoystick(steeringJoystick)
{
    AddRequirements(&m_drivetrain);
    SetName("DriveTeleopCommand");
}
void DriveTeleopCommand::Execute()
{
    double rawTranslationX = frc::ApplyDeadband(m_translationJoystick.GetY(), 0.025);
    double rawTranslationY = frc::ApplyDeadband(m_translationJoystick.GetX(), 0.025);
    rawTranslationX *= rawTranslationX * wpi::sgn(rawTranslationX);
    rawTranslationY *= rawTranslationY * wpi::sgn(rawTranslationY);

    // Normalize input so that throttle does not run faster when moving diagonally
    double translationAngle = atan2(rawTranslationY, rawTranslationX);
    double translationX = rawTranslationX * cos(translationAngle) * wpi::sgn(rawTranslationX);
    double translationY = rawTranslationY * sin(translationAngle) * wpi::sgn(rawTranslationY);

    double steeringX = frc::ApplyDeadband(m_steeringJoystick.GetX(), 0.05);
    steeringX *= steeringX * wpi::sgn(steeringX);

    if (m_translationJoystick.GetRawButton(1))
    {
        m_drivetrain.Drive(frc::ChassisSpeeds{
            -translationX * Constants::SwerveModule::ThrottleMaxVelocity,
            -translationY * Constants::SwerveModule::ThrottleMaxVelocity,
            -steeringX * Constants::SwerveModule::SteeringMaxVelocity});
    }
    else
    {
        m_drivetrain.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            -translationX * Constants::SwerveModule::ThrottleMaxVelocity,
            -translationY * Constants::SwerveModule::ThrottleMaxVelocity,
            -steeringX * Constants::SwerveModule::SteeringMaxVelocity,
            m_drivetrain.GetMeasuredRotation()));
    }
}