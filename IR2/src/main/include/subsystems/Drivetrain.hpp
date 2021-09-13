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
    void UpdateOdometry();

    // Getters
    frc::Rotation2d GetYaw();
    units::radians_per_second_t GetYawRate();

    static constexpr units::meters_per_second_t kMaxSpeedLinear = 16_fps;
    static constexpr units::radians_per_second_t kMaxSpeedAngular = 360_deg_per_s;
    static constexpr units::inch_t kWheelToWheel = 22_in;

private:
    bool m_fieldRelative;

    // Configuration
    static constexpr auto dist = kWheelToWheel / 2;
    frc::Translation2d frontLeftLocation{+dist, +dist};
    frc::Translation2d frontRightLocation{+dist, -dist};
    frc::Translation2d backLeftLocation{-dist, +dist};
    frc::Translation2d backRightLocation{-dist, -dist};

    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kZ,
        frc::SPI::Port::kOnboardCS0,
        frc::ADIS16470CalibrationTime::_1s};

    // Odomoetry
    frc::Field2d m_fieldDisplay;

    // Control
    frc::ChassisSpeeds m_command;

    static constexpr auto kMaxModuleLinearAcceleration = 80.0_mps_sq;
    static constexpr auto kMaxModuleLinearJerk = 200.0_mps_sq / 1_s;

    static constexpr auto kMaxModuleAngularVelocity = 18_rad_per_s;
    static constexpr auto kMaxModuleAngularAcceleration = 200_rad_per_s_sq;

    static constexpr SwerveModuleConfig m_frontLeftConfig{
        units::degree_t(-172.266),
        {2.42,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {8.0, // 2.5179,
         0.0, // 0.0,
         0.5, // 0.15272,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.673_V,
         2.35_V / 1_mps,
         0.0937_V / 1_mps_sq},
        {0.49655_V,
         0.65857_V / 1_rad_per_s,
         0.042166_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_frontRightConfig{
        units::degree_t(34.717),
        {2.42,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {8.0, // 4.2946,
         0.0, // 0.0,
         0.5, // 0.050889,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.673_V,
         2.35_V / 1_mps,
         0.0937_V / 1_mps_sq},
        {0.58739_V,
         0.64399_V / 1_rad_per_s,
         0.018826_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_backLeftConfig{
        units::degree_t(62.666),
        {2.42,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {8.0, // 4.9251,
         0.0, // 0.0,
         0.5, // 0.048966,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.673_V,
         2.35_V / 1_mps,
         0.0937_V / 1_mps_sq},
        {1.0045_V,
         0.6584_V / 1_rad_per_s,
         0.015321_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_backRightConfig{
        units::degree_t(25.137),
        {2.42,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {8.0, // 3.5128,
         0.0, // 0.0,
         0.5, // 0.059802,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.673_V,
         2.35_V / 1_mps,
         0.0937_V / 1_mps_sq},
        {0.63069_V,
         0.6333_V / 1_rad_per_s,
         0.024316_V / 1_rad_per_s_sq}};

    // Odometry
    frc::SwerveDriveKinematics<4> m_kinematics{
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation};

    frc::SwerveDriveOdometry<4> m_odometry{
        m_kinematics,
        frc::Rotation2d(),
        frc::Pose2d()};

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        frc::Rotation2d(),
        frc::Pose2d(),
        m_kinematics,
        {0.5, 0.5, 0.05},
        {2.0},
        {0.0, 0.0, 0.0}};

    frc::ChassisSpeeds m_robotVelocity;

    // Swerve Modules
    SwerveModule m_frontLeft{"FL", 0, 1, 2, m_frontLeftConfig};
    SwerveModule m_frontRight{"FR", 2, 3, 3, m_frontRightConfig};
    SwerveModule m_backLeft{"BL", 4, 5, 4, m_backLeftConfig};
    SwerveModule m_backRight{"BR", 6, 7, 5, m_backRightConfig};

    // Trajectory Following
    frc::HolonomicDriveController m_trajectoryController{
        frc2::PIDController{2.0, 0.0, 0.0},                      // X-error
        frc2::PIDController{2.0, 0.0, 0.0},                      // Y-error
        frc::ProfiledPIDController<units::radian>{1.0, 0.0, 0.0, // Rotation-error
                                                  frc::TrapezoidProfile<units::radian>::Constraints{
                                                      360_deg_per_s,
                                                      720_deg_per_s / 1_s}}};
};