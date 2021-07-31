#pragma once

// Utilities
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "Subsystem.hpp"

class Drivetrain : public Subsystem
{
public:
    Drivetrain();

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Setters


private:
    
};