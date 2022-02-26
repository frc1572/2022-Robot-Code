// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include "commands/ActuatorCommand.h"
#include "commands/DriveTeleopCommand.h"
#include "commands/FlywheelSpinupCommand.h"

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here
    m_drivetrain.SetDefaultCommand(DriveTeleopCommand(m_drivetrain, m_translationJoystick, m_steeringJoystick));
    // m_actuators.SetDefaultCommand(ActuatorCommand(0.0, m_actuators));
    //  Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
    // Configure your button bindings here

    // Set Speeds for the Turret Shooter
    frc2::JoystickButton(&m_joystick, 10).WhenPressed(FlywheelSpinupCommand(0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 5)
        .WhenPressed(FlywheelSpinupCommand(Constants::Systemspeeds::HoodSpeed, m_flywheel));

    // Turret Feeder (trigger is held on, and joystick left and right button are on/ off)
    frc2::JoystickButton(&m_joystick, 1)
        .WhenHeld(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel));
    frc2::JoystickButton(&m_joystick, 1).WhenReleased(FeederSpinupCommand(0.0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 4)
        .WhenPressed(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel));
    frc2::JoystickButton(&m_joystick, 3).WhenPressed(FeederSpinupCommand(0.0, m_flywheel));

    // Actuator Set Positions  (Right Top 3 buttons, Left .25, Middle .5, Right .75)
    frc2::JoystickButton(&m_joystick, 13).WhenPressed(ActuatorCommand(0.25, m_actuators));
    frc2::JoystickButton(&m_joystick, 12).WhenPressed(ActuatorCommand(0.50, m_actuators));
    frc2::JoystickButton(&m_joystick, 11).WhenPressed(ActuatorCommand(0.75, m_actuators));

    // Main Robot Feeder on and off buttons Left side of controller middle top and bottom buttons
    frc2::JoystickButton(&m_joystick, 9).WhenPressed(MainFeederSpinupCommand(0.0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 6)
        .WhenPressed(MainFeederSpinupCommand(Constants::Systemspeeds::IntakeSpeed, m_flywheel));

    // Intake on and off buttons Left side of controller right top and bottom buttons
    frc2::JoystickButton(&m_joystick, 8).WhenPressed(IntakeSpinupCommand(0.0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 7)
        .WhenPressed(IntakeSpinupCommand(Constants::Systemspeeds::IntakeFeederSpeed, m_flywheel));

    // Main Feeder, Turret Feeder, Intake, All On (16 Right Side bottom Right is On) And Off (14 Right Side bottom left
    // is off) Command
    frc2::JoystickButton(&m_joystick, 14).WhenPressed(IntakeSpinupCommand(0.0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 14).WhenPressed(MainFeederSpinupCommand(0.0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 14).WhenPressed(FeederSpinupCommand(0.0, m_flywheel));

    frc2::JoystickButton(&m_joystick, 16)
        .WhenPressed(IntakeSpinupCommand(Constants::Systemspeeds::IntakeSpeed, m_flywheel));
    frc2::JoystickButton(&m_joystick, 16)
        .WhenPressed(MainFeederSpinupCommand(Constants::Systemspeeds::IntakeFeederSpeed, m_flywheel));
    frc2::JoystickButton(&m_joystick, 16)
        .WhenPressed(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel));
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