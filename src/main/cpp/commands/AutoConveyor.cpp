#include "commands/AutoConveyor.h"
#
AutoConveyor::AutoConveyor(
    IntakeFeederSubsystem& Conveyor,
    VisionSubsystem& Vision,
    DriveTrainSubsystem& DriveTrain,
    FlywheelSubsystem& Flywheel)
  : m_conveyor(Conveyor), m_vision(Vision), m_drivetrain(DriveTrain), m_flywheel(Flywheel)
{
    AddRequirements(&m_conveyor);
}

void AutoConveyor::Execute()
{
    // 50 rpm  = 5.235 rad per s
    auto RPMOffset = m_flywheel.GetMeasuredVelocity() - m_flywheel.GetDesiredVelocity();
    if (units::math::abs(RPMOffset) < 150 * 2_rad * wpi::numbers::pi / 60_s)
    {
        if (auto targetInfo = m_vision.GetLatestResult())
        {
            auto distance = targetInfo->distance;
            m_conveyor.StartIntakeFeeder(m_ConveyorLUT.Lookup(distance));
        }
        else
        {
            auto distance = (Constants::GoalTranslation - m_drivetrain.GetPose().Translation()).Norm();
            m_conveyor.StartIntakeFeeder(m_ConveyorLUT.Lookup(distance));
        }
    }
    else
    {
        m_conveyor.StartIntakeFeeder(0);
    }
}

void AutoConveyor::End(bool Interuptable)
{
    m_conveyor.StartIntakeFeeder(0);
}