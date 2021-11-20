#pragma once

// Utilities
#include <ctre/Phoenix.h>
#include "frc/smartdashboard/SmartDashboard.h"

#define VOLTAGE (frc::SmartDashboard::GetNumber("Spindexer Target Voltage", 0.0))

class Spindexer
{
public:
    enum State
    {
        Idle = 0,
        Feed,
        F_A_S_T,
        Reverse,
        Custom
    };

    // Constructor
    Spindexer();

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Setters
    void SetState(State state);
    void Set(double speed);

private:
    WPI_TalonFX spindexerMotor{9};

    static constexpr int kEncoderResolution = 2048;
    static constexpr double kGearboxRatio = 37.5;

    static constexpr double kScaleFactor = 1.0 / (kEncoderResolution * kGearboxRatio);
};