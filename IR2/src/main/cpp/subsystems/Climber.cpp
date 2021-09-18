#include "subsystems/Climber.hpp"

Climber::Climber()
{
}

void Climber::UpdateTelemetry()
{

}

void Climber::ConfigureMotors()
{
    
}

void Climber::SetClimber(double speed)
{
    climb._Set(speed);

    // if(std::abs(speed) > 0.0)
    // {
    //     climbLock.Set(false);
    // }
    // else
    // {
    //     climbLock.Set(true);
    // }
}

void Climber::SetClimberPosition(State state)
{
    switch(state)
    {
        case State::Stowed:
        {
            climbPiston.Set(false);
            break;
        }

        case State::Deployed:
        {
            climbPiston.Set(true);
            break;
        }
    }
}