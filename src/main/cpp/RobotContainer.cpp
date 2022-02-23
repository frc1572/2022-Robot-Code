// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include "commands/DriveTeleopCommand.h"
#include "commands/FlywheelSpinupCommand.h"

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here
    m_drivetrain.SetDefaultCommand(DriveTeleopCommand(m_drivetrain, m_translationJoystick, m_steeringJoystick));
    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
    // Configure your button bindings here
    frc2::JoystickButton(&m_joystick, 2).WhenPressed(FlywheelSpinupCommand(0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 5).WhenPressed(FlywheelSpinupCommand(1000, m_flywheel));
    frc2::JoystickButton(&m_joystick, 6).WhenPressed(FlywheelSpinupCommand(2000, m_flywheel));
    frc2::JoystickButton(&m_joystick, 7).WhenPressed(FlywheelSpinupCommand(3000, m_flywheel));
    frc2::JoystickButton(&m_joystick, 8).WhenPressed(FlywheelSpinupCommand(6000, m_flywheel));
    frc2::JoystickButton(&m_joystick, 9).WhenPressed(FlywheelSpinupCommand(5000, m_flywheel));
    frc2::JoystickButton(&m_joystick, 10).WhenPressed(FlywheelSpinupCommand(4000, m_flywheel));
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    // An example command will be run in autonomous
    return nullptr;
}

void RobotContainer::Reset()
{
    m_drivetrain.Reset();
}