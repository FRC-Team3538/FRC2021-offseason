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

    // Constructor
    Intake();

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Setters
    void SetPosition(Position position);
    void SetSpeed(double speed);

    // Getters
    Position GetPosition();
    double GetSpeed();

private:
    double currentSpeed = 0.0;
    Position currentPosition = Position::Stowed;

    LazyTalonFX intakeMotor{8};

    frc::Solenoid deployPiston{0};
};