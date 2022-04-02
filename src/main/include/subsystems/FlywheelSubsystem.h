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

class FlywheelSubsystem : public frc2::SubsystemBase
{
public:
    FlywheelSubsystem();
    void Periodic() override;
    void SimulationPeriodic() override;
    void SetSetpoint(rad_per_s_t setpoint);
    rad_per_s_t GetDesiredVelocity();
    rad_per_s_t GetEstimatedVelocity();

    // void SetHoodSpeed(rad_per_s_t SelectedHoodSpeed);
    // frc2::SequentialCommandGroup HoodShot(rad_per_s_t HoodRPM,  double FeedRPM);

private:
    WPI_TalonFX m_leader{Constants::Flywheel::LeaderID, "canivore"};
    WPI_TalonFX m_follower{Constants::Flywheel::FollowerID, "canivore"};

    frc::LinearSystem<1, 1, 1> m_plant =
        frc::LinearSystemId::IdentifyVelocitySystem<units::radians>(Constants::Flywheel::Kv, Constants::Flywheel::Ka);

    frc::LinearQuadraticRegulator<1, 1> m_controller = frc::LinearQuadraticRegulator{
        m_plant,
        {0.01}, // radians/s
        {12},   // volts
        Constants::LoopPeriod};

    frc::KalmanFilter<1, 1, 1> m_filter = frc::KalmanFilter{
        m_plant,
        {200},  // model error stddev
        {1.67}, // measurement error stddev
        Constants::LoopPeriod};

    frc::LinearSystemLoop<1, 1, 1> m_loop =
        frc::LinearSystemLoop{m_plant, m_controller, m_filter, 12_V, Constants::LoopPeriod};

    frc::sim::LinearSystemSim<1, 1, 1> m_plantSim = frc::sim::LinearSystemSim{m_plant, {5}};
};