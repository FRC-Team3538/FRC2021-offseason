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
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <iostream>

struct SwerveModuleDrivePIDConfig
{
    const double kP;
    const double kI;
    const double kD;
    const units::meters_per_second_squared_t max_acceleration;
    const decltype(1_mps_sq / 1_s) max_jerk;
};

struct SwerveModuleDriveFFConfig
{
    const units::volt_t kS;
    const decltype(1_V / 1_mps) kV;
    const decltype(1_V / 1_mps_sq) kA;
};

struct SwerveModuleTurnPIDConfig
{
    const double kP;
    const double kI;
    const double kD;
    const units::radians_per_second_t max_angular_velocity;
    const units::radians_per_second_squared_t max_angular_acceleration;
};

struct SwerveModuleTurnFFConfig
{
    const units::volt_t kS;
    const decltype(1_V / 1_rad_per_s) kV;
    const decltype(1_V / 1_rad_per_s_sq) kA;
};

struct SwerveModuleConfig
{
    const units::degree_t angleOffset;
    const SwerveModuleDrivePIDConfig drivePID;
    const SwerveModuleTurnPIDConfig turningPID;
    const SwerveModuleDriveFFConfig driveFf;
    const SwerveModuleTurnFFConfig turnFf;
};

class SwerveModule : public Subsystem
{
public:
    SwerveModule(std::string moduleID, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, SwerveModuleConfig config);
    SwerveModule() = delete; // Removes default constructor because why tf u using the default constructor, my G. SMH

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

    std::string moduleID;

    // Hardware
    LazyTalonFX m_driveMotor;
    LazyTalonFX m_turningMotor;
    CANCoder turningEncAbs;

    // Configuration
    static constexpr auto kWheelRadius = 2.0_in;
    static constexpr int kEncoderResolution = 2048;
    static constexpr double kDriveGearboxRatio = 6.75;
    static constexpr double kTurnGearboxRatio = 12.8;

    static constexpr auto kDriveScaleFactor =
        (2 * wpi::math::pi * kWheelRadius) / (kDriveGearboxRatio * kEncoderResolution);

    static constexpr auto kTurningMotorVoltageNominal = 12.8_V;

    static constexpr auto kDriveMotorCurrentLimit = 55_A;
    static constexpr auto kTurningMotorCurrentLimit = 30_A;

    // Control
    frc::ProfiledPIDController<units::meters_per_second> m_drivePIDController;
    frc::ProfiledPIDController<units::radians> m_turningPIDController;

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward;
    frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward;

    // Preferences
    frc::Preferences *prefs = frc::Preferences::GetInstance();
};