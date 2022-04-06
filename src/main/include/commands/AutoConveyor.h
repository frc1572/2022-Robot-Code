#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include "frc/filter/LinearFilter.h"
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
    void Initialize() override;
    void Execute() override;
    void End(bool Interuptable) override;

private:
    IntakeFeederSubsystem& m_conveyor;
    VisionSubsystem& m_vision;
    DriveTrainSubsystem& m_drivetrain;
    FlywheelSubsystem& m_flywheel;
    rad_per_s_t PreviousVelocity;
    rad_per_s_t CurrentVelocity;

    LinearLookupTable<units::meter_t, double> m_ConveyorLUT{{
        {2_m, 0.65},
        {2.5_m, 0.65},
        {3_m, 0.80},
        {3.5_m, 0.80},
    }};

    frc::LinearFilter<rad_per_s_t> m_filter = frc::LinearFilter<rad_per_s_t>::MovingAverage(25);
};