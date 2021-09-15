#pragma once

#include "subsystems/Drivetrain.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/Shooter.hpp"
#include "subsystems/RJVisionPipeline.hpp"

class Robotmap
{
public:
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;

    vision::RJVisionPipeline vis;

    void UpdateTelemetry()
    {
        switch(telemetryCt)
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

private:
    int telemetryCt = 0;
};