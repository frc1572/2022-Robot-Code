#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class IntakeFeederSubsystem : public frc2::SubsystemBase
{
public:
    IntakeFeederSubsystem();
    void Periodic() override;
    void StartIntakeFeeder(double IntakeFeederRpm);

private:
    WPI_TalonFX m_intakeFeeder{Constants::IntakeSystem::MainFeederID, "canivore"};
};