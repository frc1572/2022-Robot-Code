#include "subsystems/FlywheelSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

FlywheelSubsystem::FlywheelSubsystem() {
    m_leader.ConfigFactoryDefault();
    // Disable Talon FX velocity filtering so that our Kalman filter can do the work
    m_leader.ConfigVelocityMeasurementPeriod(SensorVelocityMeasPeriod::Period_1Ms);
    m_leader.ConfigVelocityMeasurementWindow(1);

    frc::SmartDashboard::PutNumber("Flywheel.Setpoint", 0);
}

void FlywheelSubsystem::Periodic() {
    rad_per_s_t velocity = m_leader.GetSelectedSensorVelocity() / Constants::TicksPerRevolution::TalonFX / Constants::VelocityFactor::TalonFX;
    m_loop.Correct(frc::MakeMatrix<1, 1>(velocity.to<double>()));
    m_loop.Predict(Constants::LoopPeriod);
    m_leader.SetVoltage(Constants::Flywheel::Ks + m_loop.U(0) * 1_V);

    frc::SmartDashboard::PutNumber("Flywheel.MeasuredState", velocity.to<double>());
    frc::SmartDashboard::PutNumber("Flywheel.EstimatedState", m_loop.Xhat(0));
}

void FlywheelSubsystem::SimulationPeriodic() {
    m_plantSim.SetInput(0, m_leader.GetMotorOutputVoltage());
    m_plantSim.Update(Constants::LoopPeriod);
    auto output = m_plantSim.GetOutput(0) * 1_rad_per_s * Constants::TicksPerRevolution::TalonFX;
    auto leaderSim = m_leader.GetSimCollection();
    leaderSim.SetIntegratedSensorVelocity(output * Constants::VelocityFactor::TalonFX);
    leaderSim.AddIntegratedSensorPosition(output * Constants::LoopPeriod);
}

void FlywheelSubsystem::SetSetpoint(rad_per_s_t setpoint) {
    m_loop.SetNextR(frc::MakeMatrix<1, 1>(setpoint.to<double>()));

    frc::SmartDashboard::PutNumber("Flywheel.Setpoint", setpoint.to<double>());
}