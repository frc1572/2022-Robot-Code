#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class IntakeSystemSubsystem : public frc2::SubsystemBase
{
public:
    IntakeSystemSubsystem();
    void Periodic() override;
    void StartIntake(double IntakeRpm);
    void StartIntakeFeeder(double IntakeFeederRpm);

private:
    WPI_TalonFX m_intake{Constants::IntakeSystem::IntakeID, "canivore"};
    WPI_TalonFX m_intakeFeeder{Constants::IntakeSystem::MainFeederID, "canivore"};
};