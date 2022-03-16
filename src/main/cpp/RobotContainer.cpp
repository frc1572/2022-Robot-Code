// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include "commands/ActuatorCommand.h"
#include "commands/AutoFlywheelCommand.h"
#include "commands/AutoTurretCommand.h"
#include "commands/DriveTeleopCommand.h"
#include "commands/FlywheelSpinupCommand.h"
#include "commands/IntakeSystemCommand.h"
#include "commands/TurretCommand.h"
#include "Constants.h"
#include "frc2/command/button/POVButton.h"

RobotContainer::RobotContainer()
{
    m_drivetrain.SetDefaultCommand(DriveTeleopCommand(m_drivetrain, m_translationJoystick, m_steeringJoystick));
    // m_flywheel.SetDefaultCommand(AutoFlywheelCommand(m_drivetrain, m_flywheel));
    m_turret.SetDefaultCommand(AutoTurretCommand(m_drivetrain, m_turret));
    m_poseEstimatorCommand.Schedule();
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
    // Hood Shooter Toggle ON
    frc2::JoystickButton(&m_joystick, 6)
        .WhenPressed(FlywheelSpinupCommand(Constants::Systemspeeds::HoodSpeed, m_flywheel));
    frc2::JoystickButton(&m_joystick, 5).WhenPressed(FlywheelSpinupCommand(0, m_flywheel));

    // Turret Feededr toggle ON
    frc2::JoystickButton(&m_joystick, 6)
        .WhenHeld(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel));
    frc2::JoystickButton(&m_joystick, 5).WhenReleased(FeederSpinupCommand(0.0, m_flywheel));

    // Main Intake and Main Feeder Hold ON
    frc2::JoystickButton(&m_joystick, 3)
        .WhenHeld(IntakeFeederSpinupCommand(Constants::Systemspeeds::IntakeFeederSpeed * -1, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 3)
        .WhenHeld(IntakeSpinupCommand(-Constants::Systemspeeds::IntakeSpeed, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 3).WhenReleased(IntakeFeederSpinupCommand(0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 3).WhenReleased(IntakeSpinupCommand(0, m_intakeSystem));

    // Intake Reverse
    frc2::JoystickButton(&m_joystick, 1)
        .WhenHeld(IntakeSpinupCommand(-Constants::Systemspeeds::IntakeSpeed, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 1).WhenReleased(IntakeSpinupCommand(0, m_intakeSystem));

    // Actuator Testiong Set Positions
    frc2::JoystickButton(&m_joystick, 9).WhenPressed(ActuatorCommand(0.25, m_actuators));
    frc2::JoystickButton(&m_joystick, 10).WhenPressed(ActuatorCommand(0.50, m_actuators));
    frc2::JoystickButton(&m_joystick, 60).WhenPressed(ActuatorCommand(0.75, m_actuators));

    // Main Robot Feeder Hold On
    frc2::JoystickButton(&m_joystick, 4).WhenReleased(IntakeFeederSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 4)
        .WhenHeld(IntakeFeederSpinupCommand(Constants::Systemspeeds::IntakeFeederSpeed, m_intakeSystem));

    // Intake Hold ON
    frc2::JoystickButton(&m_joystick, 2).WhenReleased(IntakeSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 2)
        .WhenHeld(IntakeSpinupCommand(Constants::Systemspeeds::IntakeFeederSpeed, m_intakeSystem));

    // All systems on
    frc2::JoystickButton(&m_joystick, 7).WhenPressed(IntakeSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 7).WhenPressed(IntakeFeederSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 7).WhenPressed(FeederSpinupCommand(0.0, m_flywheel));
    // All systems off
    frc2::JoystickButton(&m_joystick, 8)
        .WhenPressed(IntakeFeederSpinupCommand(Constants::Systemspeeds::IntakeSpeed, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 8)
        .WhenPressed(IntakeSpinupCommand(Constants::Systemspeeds::IntakeFeederSpeed, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 8)
        .WhenPressed(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel));
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    // An example command will be run in autonomous
    return &m_testAutoCommand;
}

void RobotContainer::Reset()
{
    m_drivetrain.Reset();
}