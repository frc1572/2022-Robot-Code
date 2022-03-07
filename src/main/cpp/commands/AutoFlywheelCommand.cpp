#include "commands/AutoFlywheelCommand.h"

#include <units/angle.h>
#include <units/time.h>
#include <wpi/numbers>

AutoFlywheelCommand::AutoFlywheelCommand(DriveTrainSubsystem& drivetrain, FlywheelSubsystem& flywheel)
  : m_drivetrain(drivetrain), m_flywheel(flywheel)
{
    AddRequirements(&m_flywheel);
    SetName("AutoFlywheelCommand");
}

void AutoFlywheelCommand::Execute()
{
    auto distance = (Constants::GoalTranslation - m_drivetrain.GetPose().Translation()).Norm();
    // Convert RPM to rad/s
    m_flywheel.SetSetpoint(m_lut.Lookup(distance) * 2_rad * wpi::numbers::pi / 60_s);
}

void AutoFlywheelCommand::End(bool interrupted)
{
    m_flywheel.SetSetpoint(0_rad / 1_s);
}