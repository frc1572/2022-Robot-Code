#include "subsystems/ActuatorSubsystem.h"

#include "frc/MathUtil.h"
#include "frc/Servo.h"
#include "frc/Timer.h"

ActuatorSubsystem::ActuatorSubsystem(/*int LeftActuatorPort, int RightActuatorPort*/)
/*: m_leftActuator(LeftActuatorPort), m_rightActuator(RightActuatorPort)))*/
{
    // Settings similar to config default and brake mode go here if needed for Actuators
}
// void ActuatorSubsystem::SetActuatorPoistion()
//{
//     // m_left.SetPosition(m_ActuatorJoystick.GetZ());
//     // m_right.SetPosition(m_ActuatorJoystick.GetZ());
// }

void ActuatorSubsystem::SetActuatorPosition(double TargetPosition)
{
    m_leftActuator.SetPosition(TargetPosition);
    m_rightActuator.SetPosition(TargetPosition);
}

void ActuatorSubsystem::Periodic()
{
    //

    // The Sample code VVV (Will work on more, just ran out of time. Also I know the use of // on all the lines is
    // horrid lol)

    // public:
    //     class LinearServo extends Servo
    //     {
    //         double m_speed;
    //         double m_length;
    //         double setPos;
    //         double curPos;
    //         /**
    //          * Parameters for L16-R Actuonix Linear Actuators
    //          *
    //          * @param channel PWM channel used to control the servo
    //          * @param length max length of the servo [mm]
    //          * @param speed max speed of the servo [mm/second]
    //          */
    //         // public:
    //         LinearServo(int channel, int length, int speed)
    //         {
    //             super(channel);
    //             setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    //             m_length = length;
    //             m_speed = speed;
    //         }
    //         /**
    //          * * Run this method in any periodic function to update the position estimation of your
    //        servo
    //         *
    //         * @param setpoint the target position of the servo [mm]
    //         */
    //         // public:
    //         void setPosition(double setpoint)
    //         {
    //             setPos = MathUtil.clamp(setpoint, 0, m_length);
    //             setSpeed((setPos / m_length * 2) - 1);
    //         }
    //         double lastTime = 0;
    //         /**
    //         * Run this method in any periodic function to update the position estimation of your
    //        servo
    //         */
    //         // public:
    //         void updateCurPos()
    //         {
    //             double dt = Timer.getFPGATimestamp() - lastTime;
    //             if (curPos > setPos + m_speed * dt)
    //             {
    //                 curPos -= m_speed * dt;
    //             }
    //             else if (curPos < setPos - m_speed * dt)
    //             {
    //                 curPos += m_speed * dt;
    //             }
    //             else
    //             {
    //                 curPos = setPos;
    //             }
    //         }
    //         /**
    //         * Current position of the servo, must be calling {@link #updateCurPos()
    //        updateCurPos()} periodically
    //         *
    //         * @return Servo Position [mm]
    //         */
    //         // public:
    //         double getPosition()
    //         {
    //             return curPos;
    //         }
    //         /**
    //         * Checks if the servo is at its target position, must be calling {@link #updateCurPos()
    //        updateCurPos()} periodically
    //         * @return true when servo is at its target
    //         */
    //         // public:
    //         boolean isFinished()
    //         {
    //             return curPos == setPos;
    //         };
    //     }
}