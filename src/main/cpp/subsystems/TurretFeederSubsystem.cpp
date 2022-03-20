#include "subsystems/TurretFeederSubsystem.h"

#include <Eigen/Core>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>

#include "ctre/Phoenix.h"

TurretFeederSubsystem::TurretFeederSubsystem()
{
    // Disable Talon FX velocity filtering so that our Kalman filter can do the
    // work

    m_feeder.SetNeutralMode(Brake);
    m_feeder.ConfigFactoryDefault();
    // Set Feeder to coast and config Default
    // frc::SmartDashboard::PutNumber("Flywheel.Setpoint", 0);
}

void TurretFeederSubsystem::StartFeeder(double FeedRpm)
{
    m_feeder.Set(ControlMode::PercentOutput, FeedRpm);
}

/*
frc2::SequentialCommandGroup FlywheelSubsystem::HoodShot(rad_per_s_t HoodRPM,  double FeedRPM) {
        [this](rad_per_s_t HoodRPM, double FeedRPM) {
            FlywheelSubsystem::StartFeeder(FeedRPM);
            FlywheelSubsystem::SetSetpoint(HoodRPM);
        }

        m_feeder.Set(ControlMode::PercentOutput, FeedRPm),
        FlywheelSubsystem::SetSetpoint(HoodRPM)
}
*/