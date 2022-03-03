// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <pathplanner/lib/PathPlanner.h>
#include <units/angle.h>
#include <units/length.h>

#include "commands/PoseEstimatorCommand.h"
#include "subsystems/ActuatorSubsystem.h"
#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/IntakeSystemSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/VisionSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
    void Reset();

private:
    frc::Joystick m_joystick{1};
    frc::Joystick m_translationJoystick{3};
    frc::Joystick m_steeringJoystick{2};

    // The robot's subsystems and commands are defined here...
    DriveTrainSubsystem m_drivetrain;
    FlywheelSubsystem m_flywheel;
    // Added the actuators as a subsystem/class, aswell as used the flywheel subsystem/class for the feeder
    TurretSubsystem m_turretSystem;
    ActuatorSubsystem m_actuators;
    VisionSubsystem m_vision{
        35.3498_in,
        102.6_in,
        32_deg,
        [this]()
        {
            auto dtPose = m_drivetrain.GetPose();
            auto turretRotation = m_turretSystem.GetMeasuredPosition();
            auto cameraOffset = frc::Translation2d{Constants::CameraRotationRadius, 0_m}.RotateBy(turretRotation);
            return frc::Pose2d{dtPose.Translation() + cameraOffset, dtPose.Rotation() + turretRotation};
        }};

    PoseEstimatorCommand m_poseEstimatorCommand{m_drivetrain, m_turretSystem, m_vision};

    frc2::SequentialCommandGroup m_testAutoCommand = m_drivetrain.MakeDrivePathPlannerCommand(
        "testAutoCommmand", pathplanner::PathPlanner::loadPath("testAutoCommand", 8_mps, 2_mps_sq));
    frc2::SequentialCommandGroup m_tuningRotationCommand = m_drivetrain.MakeDrivePathPlannerCommand(
        "tuningRotationCommand", pathplanner::PathPlanner::loadPath("tuningRotation", 8_mps, 2_mps_sq));
    frc2::SequentialCommandGroup m_tuningTranslationCommand = m_drivetrain.MakeDrivePathPlannerCommand(
        "tuningTranslationCommand", pathplanner::PathPlanner::loadPath("tuningTranslation", 8_mps, 2_mps_sq));
    IntakeSystemSubsystem m_intakeSystem;

    void ConfigureButtonBindings();
};
