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
    void FeederSpinupCommand(double FeedRpm, FlywheelSubsystem& feeder);
    void Initialize() override;
    bool IsFinished() override;

private:
    rad_per_s_t m_omega;
    double m_feedRpm;
    FlywheelSubsystem& m_flywheel;
};