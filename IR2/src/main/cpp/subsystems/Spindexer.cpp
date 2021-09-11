#include "subsystems/Spindexer.hpp"

Spindexer::Spindexer()
{

}

/**
 * Sets the state of the spindexer
 * 
 * @param state The desired state of the spindexer
 */
void Spindexer::SetState(State state)
{
    switch(state)
    {
        case State::Idle:
        {
            spindexerMotor._Set(0.1);
            break;
        }

        case State::Feed:
        {
            spindexerMotor._Set(0.25);
            break;
        }

        case State::F_A_S_T:
        {
            spindexerMotor._Set(1.0);
            break;
        }

        default:
        {
            spindexerMotor._Set(0.0);
        }
    }
}