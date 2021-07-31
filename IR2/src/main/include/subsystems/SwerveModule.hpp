#pragma once

// Units
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/temperature.h>
#include <units/time.h>

// Utilities
#include <cmath>
#include "lib/LazyTalonFX.hpp"
#include <frc/kinematics/SwerveModuleState.h>
#include "Subsystem.hpp"
#include <frc/Timer.h>
#include <frc/Preferences.h>
#include <string.h>
#include <frc/DutyCycleEncoder.h>

class SwerveModule : public Subsystem
{
public:
    SwerveModule(std::string moduleID, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel);

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Odometry
    frc::SwerveModuleState GetState();
    units::meters_per_second_t GetVelocity();
    frc::Rotation2d GetAngle();

    // Module Actions
    void SetModule(const frc::SwerveModuleState &state);

private:
    frc::SwerveModuleState currentState;

    frc::SwerveModuleState targetState;

    // Configuration
    static constexpr auto kWheelRadius = 1.49_in;
    static constexpr int kEncoderResolution = 2048;
    static constexpr double kDriveGearboxRatio = 5.25;
    static constexpr double kTurnGearboxRatio = 20.0;

    static constexpr auto kDriveScaleFactor =
        (2 * wpi::math::pi * kWheelRadius) / (kDriveGearboxRatio * kEncoderResolution);

    static constexpr auto kTurningMotorVoltageNominal = 12.8_V;

    static constexpr auto kDriveMotorCurrentLimit = 55_A;
    static constexpr auto kTurningMotorCurrentLimit = 30_A;

    // Preferences
    frc::Preferences *prefs = frc::Preferences::GetInstance();

    // Hardware
    LazyTalonFX m_driveMotor;
    LazyTalonFX m_turningMotor;
    frc::DutyCycleEncoder turningEncAbs;
};