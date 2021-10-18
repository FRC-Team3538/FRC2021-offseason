#pragma once

#include "subsystems/Drivetrain.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/Shooter.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/RJVisionPipeline.hpp"
#include "subsystems/Spindexer.hpp"
#include <frc/Compressor.h>

class Robotmap
{
public:
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    Climber climber;
    frc::Compressor compressor;
    Spindexer spindexer;

    vision::RJVisionPipeline vis;

    void UpdateTelemetry()
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

private:
    int telemetryCt = 0;
};