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
#include "lib/adi/ADIS16470_IMU.h"
#include <cmath>
#include <frc/smartdashboard/SendableChooser.h>

class Drivetrain : public Subsystem, 
                   public frc::Sendable,
                   public frc::SendableHelper<SwerveModule>
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
    void Stop();

    void Test(double y, double x);

    // Getters
    frc::Rotation2d GetYaw();
    units::radians_per_second_t GetYawRate();

    // Telemetry / Smartdash
    void InitSendable(frc::SendableBuilder &builder) override;

    // Simulation
    void SimPeriodic();

    // Public config values
    static constexpr units::meters_per_second_t kMaxSpeedLinear = 16_fps;
    static constexpr units::radians_per_second_t kMaxSpeedAngular = 360_deg_per_s;
    static constexpr units::meters_per_second_squared_t kMaxAccelerationLinear = units::feet_per_second_squared_t(20.0);
    static constexpr units::inch_t kWheelToWheel = 22_in;

private:
    bool m_fieldRelative;

    // Configuration
    static constexpr auto dist = kWheelToWheel / 2;
    frc::Translation2d frontLeftLocation{+dist, +dist};
    frc::Translation2d frontRightLocation{+dist, -dist};
    frc::Translation2d backLeftLocation{-dist, +dist};
    frc::Translation2d backRightLocation{-dist, -dist};

    
#ifdef __FRC_ROBORIO__
    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kZ,
        frc::SPI::Port::kOnboardCS0,
        frc::ADIS16470CalibrationTime::_1s};
#else
    // The ADI gyro is not simulator compatible on linux
    units::radian_t m_theta = 0_rad;
#endif

    // Odomoetry
    frc::Field2d m_fieldDisplay;

    // Control
    frc::ChassisSpeeds m_command;

    static constexpr auto kMaxModuleLinearAcceleration = 80.0_mps_sq;
    static constexpr auto kMaxModuleLinearJerk = 200.0_mps_sq / 1_s;

    static constexpr auto kMaxModuleAngularVelocity = 18_rad_per_s;
    static constexpr auto kMaxModuleAngularAcceleration = 200_rad_per_s_sq;

    static constexpr SwerveModuleConfig m_frontLeftConfig{
        units::degree_t(-123.135),
        {1.89,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {1.44,   // 2.5179,
         0.0,    // 0.0,
         0.0125, // 0.15272,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.607_V,
         2.2_V / 1_mps,
         0.199_V / 1_mps_sq},
        {0.776_V,
         0.232_V / 1_rad_per_s,
         0.004_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_frontRightConfig{
        units::degree_t(75.938),
        {1.89,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {1.44,   // 2.5179,
         0.0,    // 0.0,
         0.0125, // 0.15272,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.607_V,
         2.2_V / 1_mps,
         0.199_V / 1_mps_sq},
        {0.1_V,
         0.12_V / 1_rad_per_s,
         0.008_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_backLeftConfig{
        units::degree_t(-2.549),
        {1.89,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {1.44,   // 2.5179,
         0.0,    // 0.0,
         0.0125, // 0.15272,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.607_V,
         2.2_V / 1_mps,
         0.199_V / 1_mps_sq},
        {0.1_V,
         0.12_V / 1_rad_per_s,
         0.008_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_backRightConfig{
        units::degree_t(128.848),
        {1.89,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {1.44,   // 2.5179,
         0.0,    // 0.0,
         0.0125, // 0.15272,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.607_V,
         2.2_V / 1_mps,
         0.199_V / 1_mps_sq},
        {0.1_V,
         0.12_V / 1_rad_per_s,
         0.008_V / 1_rad_per_s_sq}};

    // Heading Lock
    bool m_YawLockActive = true;
    frc2::PIDController m_yawLockPID{5.0, 0.0, 0.1};
    frc::SendableChooser<std::string> yawLock;
    bool yawLockEnabled = true;

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
    SwerveModule m_frontLeft{"FL", 0, 1, 20, m_frontLeftConfig};
    SwerveModule m_frontRight{"FR", 2, 3, 21, m_frontRightConfig};
    SwerveModule m_backLeft{"BL", 4, 5, 22, m_backLeftConfig};
    SwerveModule m_backRight{"BR", 6, 7, 23, m_backRightConfig};

    // Trajectory Following
    frc::HolonomicDriveController m_trajectoryController{
        frc2::PIDController{2.0, 0.0, 0.0},                      // X-error
        frc2::PIDController{2.0, 0.0, 0.0},                      // Y-error
        frc::ProfiledPIDController<units::radian>{2.0, 0.0, 0.0, // Rotation-error
                                                  frc::TrapezoidProfile<units::radian>::Constraints{
                                                      360_deg_per_s,
                                                      720_deg_per_s / 1_s}}};
};