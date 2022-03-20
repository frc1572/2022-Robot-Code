#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "CustomUnits.h"
#include "subsystems/TurretFeederSubsystem.h"



class FeederSpinupCommand : public frc2::CommandHelper<frc2::CommandBase, FeederSpinupCommand>
{
public:
    FeederSpinupCommand(double FeederRPM, TurretFeederSubsystem& feeder);
    void Initialize() override;
    bool IsFinished() override;

private:
    double m_feederRPM;
    TurretFeederSubsystem& m_feeder;
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