#pragma once

// Lower Level Robot Stuffs
#include "SwerveModule.hpp"

// Utilities
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/HolonomicDriveController.h>
#include "Subsystem.hpp"
#include <frc/geometry/Translation2d.h>
#include "adi/ADIS16470_IMU.h"

class Drivetrain : public Subsystem
{
public:
    Drivetrain();

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Setters
    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed,
               units::radians_per_second_t rot,
               bool fieldRelative = true);
    void Drive(frc::Trajectory::State trajectoryState, units::radian_t yaw = 0_rad);
    void ResetYaw();
    void ResetOdometry(const frc::Pose2d &pose);
    void ShowTrajectory(const frc::Trajectory &trajectory);

    // Getters
    frc::Rotation2d GetYaw();
    units::radians_per_second_t GetYawRate();

    units::meters_per_second_t kMaxSpeedLinear = 16_fps;
    units::radians_per_second_t kMaxSpeedAngular = 360_deg_per_s;
    static constexpr units::inch_t kWheelToWheel = 22_in;

private:
    // Configuration
    static constexpr auto dist = kWheelToWheel / 2;
    frc::Translation2d frontLeftLocation{+dist, +dist};
    frc::Translation2d frontRightLocation{+dist, -dist};
    frc::Translation2d backLeftLocation{-dist, +dist};
    frc::Translation2d backRightLocation{-dist, -dist};

    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kZ,
        frc::SPI::Port::kOnboardCS0,
        frc::ADIS16470CalibrationTime::_4s};

    
};