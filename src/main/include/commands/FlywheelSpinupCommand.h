#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "CustomUnits.h"
#include "subsystems/FlywheelSubsystem.h"

class FlywheelSpinupCommand : public frc2::CommandHelper<frc2::CommandBase, FlywheelSpinupCommand>
{
public:
    FlywheelSpinupCommand(rad_per_s_t omega, FlywheelSubsystem& flywheel);
    FlywheelSpinupCommand(double rpm, FlywheelSubsystem& flywheel);
    void Initialize() override;
    bool IsFinished() override;

private:
    rad_per_s_t m_omega;
    FlywheelSubsystem& m_flywheel;
};

// Created another class for feederspinupcommand, >>>Not sure if this works or is and effecient way to do it<<<
class FeederSpinupCommand : public frc2::CommandHelper<frc2::CommandBase, FeederSpinupCommand>
{
public:
    FeederSpinupCommand(double FeederRPM, FlywheelSubsystem& feeder);
    void Initialize() override;
    bool IsFinished() override;

private:
    double m_feederRPM;
    FlywheelSubsystem& m_feeder;
};

class MainFeederSpinupCommand : public frc2::CommandHelper<frc2::CommandBase, MainFeederSpinupCommand>
{
public:
    MainFeederSpinupCommand(double MainFeederRPM, FlywheelSubsystem& mainFeeder);
    void Initialize() override;
    bool IsFinished() override;

private:
    double m_mainFeederRPM;
    FlywheelSubsystem& m_mainFeeder;
};

class IntakeSpinupCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeSpinupCommand>
{
public:
    IntakeSpinupCommand(double IntakeRPM, FlywheelSubsystem& intake);
    void Initialize() override;
    bool IsFinished() override;

private:
    double m_intakeRPM;
    FlywheelSubsystem& m_intake;
};