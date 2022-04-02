// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/ActuatorCommand.h"
#include "commands/AutoConveyor.h"
#include "commands/AutoFlywheelCommand.h"
#include "commands/AutoTurretCommand.h"
#include "commands/ClimbCommand.h"
#include "commands/DriveTeleopCommand.h"
#include "commands/FlywheelSpinupCommand.h"
#include "commands/IntakeFeederCommand.h"
#include "commands/IntakeSystemCommand.h"
#include "commands/TurretCommand.h"
#include "commands/TurretFeederCommand.h"
#include "commands/TurretManualControl.h"
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/button/POVButton.h"
#include "frc2/command/CommandHelper.h"

RobotContainer::RobotContainer()
{
    frc::SmartDashboard::PutNumber("Desired Flywheel RPM:", 0);
    frc::SmartDashboard::PutNumber("Conveyor Speed (0-1): ", 0);
    m_drivetrain.SetDefaultCommand(DriveTeleopCommand(m_drivetrain, m_translationJoystick, m_steeringJoystick));

    m_turret.SetDefaultCommand(AutoTurretCommand(m_drivetrain, m_turret, m_flywheel, m_vision));
    m_climb.SetDefaultCommand(
        WinchCommand(Constants::Systemspeeds::WinchRelease, m_climb, m_translationJoystick, m_joystick));
    // m_poseEstimatorCommand.Schedule();
    //  m_autoChooser.SetDefaultOption("Left2BallAuto", &m_LeftTwoBallAuto);
    //  m_autoChooser.AddOption("Right2BallAuto", &m_RightTwoBallAuto);
    m_autoChooser.SetDefaultOption("RightOnlyReset", &m_resetRightOnly);
    m_autoChooser.AddOption("LeftOnlyReset", &m_resetLeftOnly);
    m_autoChooser.AddOption("LEFT2BALLAUTO", &m_LeftTwoBallAuto);
    m_autoChooser.AddOption("RIGHT2BALLAUTO", &m_RightTwoBallAuto);
    m_autoChooser.AddOption("Left1BallAutoNOMOVE", &m_resetLeftLowGoalShot);
    m_autoChooser.AddOption("Right1BallAutoNOMOVE", &m_resetRightLowGoalShot);
    m_autoChooser.AddOption("SmallForwardTest", &m_smallForwardAutoTest);
    m_autoChooser.AddOption("Right5ball", &m_Right5ball);
    m_autoChooser.AddOption("Right3ball", &m_Right3ball);

    frc::SmartDashboard::PutData(&m_autoChooser);
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
    // variable hood speed based on driver buttons
    // frc2::JoystickButton(&m_translationJoystick, 2).WhenPressed(SetHoodSpeed(1000));
    frc2::JoystickButton(&m_joystick, 1).WhenHeld(TurretManualControl(m_turret));
    // Hood Shooter Toggle ON
    // frc2::JoystickButton(&m_joystick, 6).WhileHeld(FlywheelSpinupCommand(0, m_flywheel));
    frc2::JoystickButton(&m_joystick, 6).WhenHeld(AutoFlywheelCommand(m_drivetrain, m_flywheel, m_vision));
    frc2::JoystickButton(&m_joystick, 6).WhenHeld(AutoConveyor(m_IntakeFeeder, m_vision, m_drivetrain, m_flywheel));
    frc2::JoystickButton(&m_joystick, 6)
        .WhenHeld(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_turretFeeder));
    // Turret Feededr toggle ON
    // frc2::JoystickButton(&m_joystick, 6)
    //    .WhenHeld(FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_turretFeeder));
    frc2::JoystickButton(&m_joystick, 6).WhenReleased(FeederSpinupCommand(0.0, m_turretFeeder));

    // Main Intake and Main Feeder Hold ON
    frc2::JoystickButton(&m_joystick, 3)
        .WhenHeld(IntakeSpinupCommand(-Constants::Systemspeeds::IntakeSpeed, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 3).WhenReleased(IntakeSpinupCommand(0, m_intakeSystem));
    // // Intake Reverse
    // frc2::JoystickButton(&m_joystick, 1)
    //     .WhenHeld(IntakeSpinupCommand(-Constants::Systemspeeds::IntakeSpeed, m_intakeSystem));
    // frc2::JoystickButton(&m_joystick, 1).WhenReleased(IntakeSpinupCommand(0, m_intakeSystem));

    // // Actuator Testiong Set Positions
    // frc2::JoystickButton(&m_joystick, 9).WhenPressed(ActuatorCommand(0.25, m_actuators));
    // frc2::JoystickButton(&m_joystick, 10).WhenPressed(ActuatorCommand(0.50, m_actuators));
    // frc2::JoystickButton(&m_joystick, 60).WhenPressed(ActuatorCommand(0.75, m_actuators));

    // Main Robot Feeder Hold On

    // frc2::JoystickButton(&m_joystick, 4).WhenHeld(IntakeFeederCommand(0, m_IntakeFeeder));
    frc2::JoystickButton(&m_joystick, 4).WhenHeld(IntakeFeederCommand(.5, m_IntakeFeeder));

    // Intake Hold ON
    frc2::JoystickButton(&m_joystick, 2).WhenReleased(IntakeSpinupCommand(0.0, m_intakeSystem));
    frc2::JoystickButton(&m_joystick, 2)
        .WhenHeld(IntakeSpinupCommand(Constants::Systemspeeds::IntakeSpeed, m_intakeSystem));

    // All systems Reverse

    frc2::JoystickButton(&m_joystick, 11).WhenHeld(IntakeFeederCommand(-1, m_IntakeFeeder));
    frc2::JoystickButton(&m_joystick, 11).WhenHeld(FeederSpinupCommand(-1, m_turretFeeder));
    frc2::JoystickButton(&m_joystick, 11)
        .WhenHeld(FlywheelSpinupCommand(-Constants::Systemspeeds::HoodSpeed, m_flywheel));
    // All systems Reverse Off
    frc2::JoystickButton(&m_joystick, 11).WhenReleased(IntakeFeederCommand(0.0, m_IntakeFeeder));

    frc2::JoystickButton(&m_joystick, 11).WhenReleased(FeederSpinupCommand(0.0, m_turretFeeder));
    frc2::JoystickButton(&m_joystick, 11).WhenReleased(FlywheelSpinupCommand(0.0, m_flywheel));
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    // An example command will be run in autonomous
    return m_autoChooser.GetSelected();
}

void RobotContainer::Reset()
{
    // m_drivetrain.Reset();
}