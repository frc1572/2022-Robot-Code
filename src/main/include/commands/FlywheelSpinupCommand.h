#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "CustomUnits.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "subsystems/FlywheelSubsystem.h"

class FlywheelSpinupCommand : public frc2::CommandHelper<frc2::CommandBase, FlywheelSpinupCommand>
{
public:
    FlywheelSpinupCommand(rad_per_s_t omega, FlywheelSubsystem& flywheel);
    FlywheelSpinupCommand(double rpm, FlywheelSubsystem& flywheel);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    rad_per_s_t m_omega;
    FlywheelSubsystem& m_flywheel;
};

/*
class SetHoodSpeed : public frc2::CommandHelper<frc2::CommandBase, SetHoodSpeed>
{
public:
    SetHoodSpeed(double SelectedHoodSpeed);
    void Initialize() override;
    bool IsFinished() override;

private:
    double m_selectedHoodSpeed;
};
*/