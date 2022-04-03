#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
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
    void SetSetpoint(rad_per_s_t setpoint);
    rad_per_s_t GetDesiredVelocity();
    rad_per_s_t GetMeasuredVelocity();
    rad_per_s_t GetFollowerMeasuredVelocity();

private:
    WPI_TalonFX m_leader{Constants::Flywheel::LeaderID, "canivore"};
    WPI_TalonFX m_follower{Constants::Flywheel::FollowerID, "canivore"};

    frc::LinearSystem<1, 1, 1> m_plant =
        frc::LinearSystemId::IdentifyVelocitySystem<units::radians>(Constants::Flywheel::Kv, Constants::Flywheel::Ka);

    frc::LinearPlantInversionFeedforward<1, 1> m_feedforward{m_plant, Constants::LoopPeriod};

    rad_per_s_t m_desiredVelocity;
};