#include "Robotmap.hpp"

void Robotmap::UpdateTelemetry()
{
    switch (telemetryCt)
    {
    case 0:
    {
        drivetrain.UpdateTelemetry();
        break;
    }
    case 1:
    {
        intake.UpdateTelemetry();
        break;
    }
    case 2:
    {
        climber.UpdateTelemetry();
        break;
    }
    case 3:
    {
        spindexer.UpdateTelemetry();
        break;
    }
    case 4:
    {
        shooter.UpdateTelemetry();
        break;
    }
    default:
    {
        telemetryCt = -1;
    }
    }
    ++telemetryCt;
}

void Robotmap::ConfigureMotors()
{
    drivetrain.ConfigureMotors();
    intake.ConfigureMotors();
    shooter.ConfigureMotors();
    climber.ConfigureMotors();
    spindexer.ConfigureMotors();
}