#pragma once

// Utilities
#include "Subsystem.hpp"
#include <ctre/Phoenix.h>
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

    WPI_TalonFX intakeMotor{8};

    frc::Solenoid deployPiston{frc::PneumaticsModuleType::CTREPCM, 0};
};