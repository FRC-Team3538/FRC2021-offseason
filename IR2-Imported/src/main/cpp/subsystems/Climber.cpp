#include "subsystems/Climber.hpp"

Climber::Climber()
{
}

void Climber::UpdateTelemetry()
{

}

void Climber::ConfigureMotors()
{
    
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    climb.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);
}

void Climber::SetClimber(double speed)
{
    climb.Set(speed);

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
            climbPiston.Set(true);
            break;
        }

        case State::Deployed:
        {
            climbPiston.Set(false);
            break;
        }
    }
}