#include "subsystems/Intake.hpp"

/**
 * The equal operator for an object of type Intake::State
 * 
 * @param param Sets the object's value to param
 * @return 
 */

Intake::Intake()
{
}

void Intake::UpdateTelemetry()
{

}

void Intake::ConfigureMotors()
{
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);
}

/**
 * Sets the state of the Intake
 * 
 * @param state Desired state of the intake (Type IntakeNS::State)
 */
void Intake::SetPosition(Position position)
{
    // Update current state for the getter function
    currentPosition = position;

    // Set piston configuration based on desired state
    switch (position)
    {

    case Position::Stowed:
    {
        deployPiston.Set(false);
        break;
    }

    case Position::Deployed:
    {
        deployPiston.Set(true);
        break;
    }

    // If inputed intake position isn't valid, default the intake to stowed
    default:
        SetPosition(Position::Stowed);
    }
}

void Intake::SetSpeed(double speed)
{
    // Min Max the intake speed
    speed = speed > 1.0 ? 1.0 : speed;
    speed = speed < -1.0 ? -1.0 : speed;

    currentSpeed = speed;

    intakeMotor._Set(speed);
}

/**
 * Returns the current position of the intake
 * 
 * @return IntakeNS::Position currentPosition
 */
Intake::Position Intake::GetPosition()
{
    return currentPosition;
}

double Intake::GetSpeed()
{
    return currentSpeed;
}