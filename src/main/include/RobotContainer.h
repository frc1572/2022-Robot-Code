// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <pathplanner/lib/PathPlanner.h>
#include <units/angle.h>
#include <units/length.h>

#include "commands/FlywheelSpinupCommand.h"
#include "commands/IntakeFeederCommand.h"
#include "commands/IntakeSystemCommand.h"
#include "commands/PoseEstimatorCommand.h"
#include "frc/Controller.h"
#include "frc/smartdashboard/SendableChooser.h"
#include "frc/smartdashboard/SendableChooserBase.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/ParallelRaceGroup.h"
#include "subsystems/ActuatorSubsystem.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/IntakeFeederSubsystem.h"
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
    frc::SendableChooser<frc2::Command*> m_autoChooser;
    // The robot's subsystems and commands are defined here...
    DriveTrainSubsystem m_drivetrain;
    FlywheelSubsystem m_flywheel;
    IntakeFeederSubsystem m_IntakeFeeder;
    ClimbSubsystem m_climb;
    IntakeSystemSubsystem m_intakeSystem;
    TurretSubsystem m_turret;
    ActuatorSubsystem m_actuators;
    VisionSubsystem m_vision{
        35.3498_in,
        102.6_in,
        32_deg,
        [this]()
        {
            auto dtPose = m_drivetrain.GetPose();
            auto turretRotation = m_turret.GetMeasuredRotation();
            auto cameraOffset = frc::Translation2d{Constants::CameraRotationRadius, 0_m}.RotateBy(turretRotation);
            return frc::Pose2d{dtPose.Translation() + cameraOffset, dtPose.Rotation() + turretRotation};
        }};

    PoseEstimatorCommand m_poseEstimatorCommand{m_drivetrain, m_turret, m_vision};

    std::function<void(frc::Pose2d)> resetPose = [this](frc::Pose2d pose)
    {
        m_drivetrain.Reset(pose.Rotation());
        m_turret.Reset({});
        m_poseEstimatorCommand.Reset(pose, m_turret.GetMeasuredRotation());
    };

    // frc2::SequentialComman9++dGroup m_testAutoCommand = m_drivetrain.MakeDrivePathPlannerCommand(
    //    "testAutoCommmand", pathplanner::PathPlanner::loadPath("testAutoCommand", 1_mps, 1_mps_sq), resetPose);
    // frc2::SequentialCommandGroup m_tuningRotationCommand = m_drivetrain.MakeDrivePathPlannerCommand(
    //     "tuningRotationCommand", pathplanner::PathPlanner::loadPath("tuningRotation", 1_mps, 1_mps_sq), resetPose);
    // frc2::SequentialCommandGroup m_tuningTranslationCommand = m_drivetrain.MakeDrivePathPlannerCommand(
    //     "tuningTranslationCommand",
    //     pathplanner::PathPlanner::loadPath("tuningTranslation", 1_mps, 1_mps_sq),
    //     resetPose);
    // frc2::SequentialCommandGroup m_left2BallCommand = m_drivetrain.MakeDrivePathPlannerCommand(
    //    "left2BallCommand", pathplanner::PathPlanner::loadPath("left2Ball", 1_mps, 1_mps_sq), resetPose);
    // frc2::SequentialCommandGroup m_right2BallCommand = m_drivetrain.MakeDrivePathPlannerCommand(
    //    "right2BallCommand", pathplanner::PathPlanner::loadPath("right2Ball", 1_mps, 1_mps_sq), resetPose);

    frc2::SequentialCommandGroup m_LeftTwoBallAuto{
        IntakeSpinupCommand(-Constants::Systemspeeds::IntakeSpeed, m_intakeSystem).WithTimeout(0.5_s),
        frc2::ParallelCommandGroup(
            IntakeSpinupCommand(Constants::Systemspeeds::IntakeSpeed, m_intakeSystem),
            IntakeFeederCommand(0.2, m_IntakeFeeder),
            frc2::SequentialCommandGroup(
                std::move(m_drivetrain.MakeDrivePathPlannerCommand(
                              "left2BallCommand",
                              pathplanner::PathPlanner::loadPath("left2Ball", 0.1_mps, 0.1_mps_sq),
                              resetPose))
                    .WithTimeout(2_s),
                FlywheelSpinupCommand(1900, m_flywheel),
                FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel)))};

    frc2::SequentialCommandGroup m_RightTwoBallAuto{
        IntakeSpinupCommand(-0.3, m_intakeSystem).WithTimeout(0.5_s),
        frc2::ParallelCommandGroup(
            IntakeSpinupCommand(Constants::Systemspeeds::IntakeSpeed, m_intakeSystem),
            IntakeFeederCommand(0.2, m_IntakeFeeder),
            frc2::SequentialCommandGroup(
                std::move(m_drivetrain.MakeDrivePathPlannerCommand(
                              "right2BallCommand",
                              pathplanner::PathPlanner::loadPath("right2Ball", 0.1_mps, 0.1_mps_sq),
                              resetPose))
                    .WithTimeout(2_s),
                FlywheelSpinupCommand(1900, m_flywheel),
                FeederSpinupCommand(Constants::Systemspeeds::TurretFeederSpeed, m_flywheel)))};

    frc2::InstantCommand m_resetLeftOnly{[this]() { resetPose({23.1548_ft, 15.4612_ft, 159.00_deg}); }};
    frc2::InstantCommand m_resetRightOnly{[this]() { resetPose({25.8845_ft, 9.3302_ft, -111.00_deg}); }};

    void ConfigureButtonBindings();
};
