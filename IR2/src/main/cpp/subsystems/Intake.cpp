#include "subsystems/Intake.hpp"

/**
 * The equal operator for an object of type Intake::State
 * 
 * @param param Sets the object's value to param
 * @return 
 */
Intake::State &Intake::State::operator=(State const &param)
{
    this->position = param.position;
    this->speed = param.speed;
}

Intake::Intake()
{
}

/**
 * Sets the state of the Intake
 * 
 * @param state Desired state of the intake (Type IntakeNS::State)
 */
void Intake::SetState(State state)
{
    // Min Max the intake speed
    state.speed = state.speed > 1.0 ? 1.0 : state.speed;
    state.speed = state.speed < -1.0 ? -1.0 : state.speed;

    // Update current state for the getter function
    currentState = state;

    // Set piston configuration based on desired state
    switch (state.position)
    {

    case Position::Stowed:
    {
        backPiston.Set(false);
        linkagePiston.Set(false);

        // Set speed of intake motor
        IntakeMotor._Set(0.0);

        break;
    }

    case Position::HumanPlayer:
    {
        backPiston.Set(true);
        linkagePiston.Set(true);

        // Set speed of intake motor
        IntakeMotor._Set(state.speed);

        break;
    }

    case Position::Deployed:
    {
        backPiston.Set(true);
        linkagePiston.Set(false);

        // Set speed of intake motor
        IntakeMotor._Set(state.speed);

        break;
    }

    // If inputed intake position isn't valid, default the intake to stowed
    default:
        SetState(State newState{state.speed, Position::Stowed});
    }
}

/**
 * Returns the current state of the intake
 * 
 * @return IntakeNS::State currentState
 */
Intake::State Intake::GetState()
{
    return currentState;
}