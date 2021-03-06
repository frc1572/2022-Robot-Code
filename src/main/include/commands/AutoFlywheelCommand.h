#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include "helper/LinearLookupTable.h"
#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class AutoFlywheelCommand : public frc2::CommandHelper<frc2::CommandBase, AutoFlywheelCommand>
{
public:
    AutoFlywheelCommand(DriveTrainSubsystem& drivetrain, FlywheelSubsystem& flywheel, VisionSubsystem& vision);
    void Execute() override;
    void End(bool interrupted) override;

private:
    DriveTrainSubsystem& m_drivetrain;
    FlywheelSubsystem& m_flywheel;
    VisionSubsystem& m_vision;

    LinearLookupTable<units::meter_t, double> m_lut{{
        {2_m, 1700},   // 1700
        {2.5_m, 1775}, // 1600
        {3_m, 2300},   // 1800
        //{3.5_m, 2300}, // 1925   // 2250
    }};
};