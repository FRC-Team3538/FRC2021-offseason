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

    //vision::RJVisionPipeline vis;

    void UpdateTelemetry();

    void ConfigureMotors();

private:
    int telemetryCt = 0;
};