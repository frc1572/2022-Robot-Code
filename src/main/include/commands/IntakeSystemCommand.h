#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSystemSubsystem.h"

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