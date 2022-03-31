#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include "helper/LinearLookupTable.h"
#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"

class AutoFlywheelCommand : public frc2::CommandHelper<frc2::CommandBase, AutoFlywheelCommand>
{
public:
    AutoFlywheelCommand(DriveTrainSubsystem& drivetrain, FlywheelSubsystem& flywheel);
    void Execute() override;
    void End(bool interrupted) override;

private:
    DriveTrainSubsystem& m_drivetrain;
    FlywheelSubsystem& m_flywheel;

    LinearLookupTable<units::meter_t, double> m_lut{{
        {1_m, 1000},
        {2_m, 1550},
        {3_m, 1900},
        {4_m, 2075},
        {5_m, 2125},
        {6_m, 2250},
        {7_m, 2600},
        {8_m, 2750},
        {9_m, 3100},
        {10_m, 3350},
    }};
};