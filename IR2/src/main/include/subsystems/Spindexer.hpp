#pragma once

// Utilities
#include "lib/LazyTalonFX.hpp"

class Spindexer
{
public:
    enum State
    {
        Idle = 0,
        Feed,
        F_A_S_T
    };

    // Constructor
    Spindexer();

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Setters
    void SetState(State state);

private:
    LazyTalonFX spindexerMotor{9};
};