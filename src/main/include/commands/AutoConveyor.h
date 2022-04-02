#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include "helper/LinearLookupTable.h"
#include "subsystems/DriveTrainSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/IntakeFeederSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class AutoConveyor : public frc2::CommandHelper<frc2::CommandBase, AutoConveyor>
{
public:
    AutoConveyor(
        IntakeFeederSubsystem& Conveyor,
        VisionSubsystem& Vision,
        DriveTrainSubsystem& DriveTrain,
        FlywheelSubsystem& flywheel);
    void Execute() override;
    void End(bool Interuptable) override;

private:
    IntakeFeederSubsystem& m_conveyor;
    VisionSubsystem& m_vision;
    DriveTrainSubsystem& m_drivetrain;
    FlywheelSubsystem& m_flywheel;

    LinearLookupTable<units::meter_t, double> m_ConveyorLUT{{
        {2_m, 0.85},
        {2.5_m, 0.65},
        {3_m, 0.65},
        {3.5_m, 0.70},
        {4_m, 0.65},
    }};
};