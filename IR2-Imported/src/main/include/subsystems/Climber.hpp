#pragma once

// Utilities
#include "lib/LazyTalonFX.hpp"
#include "subsystems/Subsystem.hpp"
#include <frc/Solenoid.h>
#include <cmath>

class Climber : public Subsystem
{
public:
    enum class State:uint8_t
    {
        Stowed = 0,
        Deployed
    };

    // Constructor
    Climber();

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Setters
    void SetClimber(double speed);
    void SetClimberPosition(State state);

    // Getters
    double GetClimberSpeed();
    State GetClimberPosition();

private:
    LazyTalonFX climb{14};

    frc::Solenoid climbPiston{1};
    // frc::Solenoid climbLock{2};
};