// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include "commands/ActuatorCommand.h"
#include "commands/DriveTeleopCommand.h"
#include "commands/FlywheelSpinupCommand.h"
#include "commands/IntakeSystemCommand.h"
#include "commands/TurretCommand.h"
#include "frc2/command/button/POVButton.h"

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

    // Set Speeds for the Turret Shooter   (Former off 10)
    frc2::JoystickButton(&m_joystick, 10).WhenPressed(FlywheelSpinupCommand(0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 5)
        .WhenPressed(FlywheelSpinupCommand(Constants::Systemspeeds::HoodSpeed, m_flywheel));

    // Turret Feeder (trigger is held on, and joystick left and right button are on/ off)
    frc2::JoystickButton(&m_joystick, 1)
        .WhenHeld(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel));
    frc2::JoystickButton(&m_joystick, 1).WhenReleased(FeederSpinupCommand(0.0, m_flywheel));
    // frc2::JoystickButton(&m_joystick, 4)
    //     .WhenPressed(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel));
    // frc2::JoystickButton(&m_joystick, 3).WhenPressed(FeederSpinupCommand(0.0, m_flywheel));

    // Actuator Set Positions  (Right Top 3 buttons, Left .25, Middle .5, Right .75)
    frc2::JoystickButton(&m_joystick, 13).WhenPressed(ActuatorCommand(0.25, m_actuators));
    frc2::JoystickButton(&m_joystick, 12).WhenPressed(ActuatorCommand(0.50, m_actuators));
    frc2::JoystickButton(&m_joystick, 11).WhenPressed(ActuatorCommand(0.75, m_actuators));

    // Main Robot Feeder on and off buttons Left side of controller middle top and bottom buttons    (former off 9)
    frc2::JoystickButton(&m_joystick, 6).WhenReleased(IntakeFeederSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 6)
        .WhenHeld(IntakeFeederSpinupCommand(Constants::Systemspeeds::IntakeSpeed, m_intakeSystem));

    // Intake on and off buttons Left side of controller right top and bottom buttons    (former off 8)
    frc2::JoystickButton(&m_joystick, 7).WhenReleased(IntakeSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 7)
        .WhenHeld(IntakeSpinupCommand(Constants::Systemspeeds::IntakeFeederSpeed, m_intakeSystem));

    // Main Feeder, Turret Feeder, Intake, All On (16 Right Side bottom Right is On) And Off (14 Right Side bottom left
    // is off) Command
    frc2::JoystickButton(&m_joystick, 14).WhenPressed(IntakeSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 14).WhenPressed(IntakeFeederSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 14).WhenPressed(FeederSpinupCommand(0.0, m_flywheel));

    frc2::JoystickButton(&m_joystick, 16)
        .WhenPressed(IntakeFeederSpinupCommand(Constants::Systemspeeds::IntakeSpeed, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 16)
        .WhenPressed(IntakeSpinupCommand(Constants::Systemspeeds::IntakeFeederSpeed, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 16)
        .WhenPressed(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel));

    // could maybe use Constants::Turret::Turretposition += 5 later for position setiing
    // frc2::JoystickButton(&m_joystick, 20).WhenHeld(TurretCommand(35, m_turretSystem));
    // frc2::JoystickButton(&m_joystick, 25).WhenHeld(TurretCommand(-35, m_turretSystem));
    // frc2::JoystickButton(&m_joystick, 20).WhenReleased(TurretCommand(00, m_turretSystem));
    // frc2::JoystickButton(&m_joystick, 25).WhenReleased(TurretCommand(-00, m_turretSystem));

    frc2::JoystickButton(&m_joystick, 4).WhenPressed(TurretCommand(1000, m_turretSystem));
    frc2::JoystickButton(&m_joystick, 3).WhenPressed(TurretCommand(-1000, m_turretSystem));
    frc2::JoystickButton(&m_joystick, 2).WhenPressed(TurretCommand(0, m_turretSystem));
    // frc2::JoystickButton(&m_joystick, 4).WhenReleased(TurretCommand(0, m_turretSystem));
    // frc2::JoystickButton(&m_joystick, 3).WhenReleased(TurretCommand(0, m_turretSystem));
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