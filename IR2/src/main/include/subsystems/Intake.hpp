#pragma once

// Utilities
#include "Subsystem.hpp"
#include "lib/LazyTalonFX.hpp"
#include <frc/Solenoid.h>

// **** INTAKE CLASS ****

class Intake : public Subsystem
{
public:
    // Data Struct
    enum class Position:uint8_t
    {
        Stowed = 0,
        Deployed
    };

    struct State
    {
        double speed;
        Position position;

        State();
        State(double speed, Position position) : speed(speed), position(position) {}

        State &operator=(State const &param);
    };

    // Constructor
    Intake();

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Setters
    void SetState(State state);

    // Getters
    State GetState();

private:
    State currentState{0.0, Position::Stowed};

    LazyTalonFX IntakeMotor{8};

    frc::Solenoid deployPiston{0};
};