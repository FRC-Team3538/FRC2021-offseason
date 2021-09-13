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