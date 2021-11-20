#pragma once

// Utilities
#include <ctre/Phoenix.h>
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
    WPI_TalonFX climb{14};

    frc::Solenoid climbPiston{frc::PneumaticsModuleType::CTREPCM, 1};
    // frc::Solenoid climbLock{frc::PneumaticsModuleType::CTREPCM, 2};
};