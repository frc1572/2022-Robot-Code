#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/Servo.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "CustomUnits.h"
#include "frc/Joystick.h"

class TurretFeederSubsystem : public frc2::SubsystemBase
{
public:
    TurretFeederSubsystem();

    void StartFeeder(double FeedRpm);
    // void SetHoodSpeed(rad_per_s_t SelectedHoodSpeed);
    // frc2::SequentialCommandGroup HoodShot(rad_per_s_t HoodRPM,  double FeedRPM);

private:
    WPI_TalonFX m_feeder{Constants::Flywheel::FeederID, "canivore"};
};