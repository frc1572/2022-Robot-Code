#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSystemSubsystem.h"

class IntakeFeederSpinupCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeFeederSpinupCommand>
{
public:
    IntakeFeederSpinupCommand(double IntakeFeederRPM, IntakeSystemSubsystem& intakeFeeder);
    void Initialize() override;
    bool IsFinished() override;

private:
    double m_intakeFeederRPM;
    IntakeSystemSubsystem& m_intakeFeeder;
};

class IntakeSpinupCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeSpinupCommand>
{
public:
    IntakeSpinupCommand(double IntakeRPM, IntakeSystemSubsystem& intake);
    void Initialize() override;
    bool IsFinished() override;

private:
    double m_intakeRPM;
    IntakeSystemSubsystem& m_intake;
};