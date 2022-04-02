#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeFeederSubsystem.h"

class IntakeFeederCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeFeederCommand>
{
public:
    IntakeFeederCommand(double IntakeFeederRPM, IntakeFeederSubsystem& intakeFeeder);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    double m_intakeFeederRPM;
    IntakeFeederSubsystem& m_intakeFeeder;
};
