#include "subsystems/Spindexer.hpp"

Spindexer::Spindexer()
{

}

void Spindexer::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber("Spindexer Target Voltage", VOLTAGE);
    frc::SmartDashboard::PutNumber("Spindexer RPS", (spindexerMotor.GetSelectedSensorVelocity(0) * kScaleFactor * 10.0));
}

void Spindexer::ConfigureMotors()
{
    spindexerMotor.SetInverted(false);
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
            spindexerMotor._Set(-0.15);
            break;
        }

        case State::Feed:
        {
            spindexerMotor._Set(-0.5);
            break;
        }

        case State::F_A_S_T:
        {
            spindexerMotor._Set(1.0);
            break;
        }

        case State::Reverse:
        {
            spindexerMotor._Set(0.5);
            break;
        }
        case State::Custom:
        {
            spindexerMotor._SetVoltage(units::volt_t{VOLTAGE});
            break;
        }

        default:
        {
            spindexerMotor._Set(0.0);
        }
    }
}

void Spindexer::Set(double speed)
{
    spindexerMotor._Set(speed);
}