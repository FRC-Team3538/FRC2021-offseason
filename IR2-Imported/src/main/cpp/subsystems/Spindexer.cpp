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

    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    spindexerMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);
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