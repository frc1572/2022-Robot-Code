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
    double rawTranslationX = frc::ApplyDeadband(m_translationJoystick.GetY(), 0.05);
    double rawTranslationY = frc::ApplyDeadband(m_translationJoystick.GetX(), 0.05);

    // Normalize input so that throttle does not run faster when moving diagonally
    // double translationAngle = atan2(rawTranslationX, rawTranslationY);
    // double translationX = rawTranslationX * cos(translationAngle) * wpi::sgn(rawTranslationX);
    // double translationY = rawTranslationY * sin(translationAngle) * wpi::sgn(rawTranslationY);

    double steeringX = frc::ApplyDeadband(m_steeringJoystick.GetX(), 0.05);

    auto robotRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        -rawTranslationX * Constants::SwerveModule::ThrottleMaxVelocity,
        rawTranslationY * Constants::SwerveModule::ThrottleMaxVelocity,
        steeringX * Constants::SwerveModule::SteeringMaxVelocity,
        m_drivetrain.GetMeasuredRotation());
    std::cout << "Robot Relative VX: " << robotRelativeSpeeds.vx.value() << std::endl;
    m_drivetrain.Drive(std::move(robotRelativeSpeeds));

    auto distance = (Constants::GoalTranslation.Norm() - m_drivetrain.GetPose().Translation().Norm());

    frc::SmartDashboard::PutNumber("Distance: ", distance.value());
    // std::cout << "Translation X Value: " << translationX << std::endl;
    // std::cout << "Raw Translation X Value: " << rawTranslationX << std::endl;
}