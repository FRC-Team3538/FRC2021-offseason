#include "subsystems/Drivetrain.hpp"

Drivetrain::Drivetrain()
{
  m_imu.Reset();
  m_imu.Calibrate();

  m_yawLockPID.EnableContinuousInput(-180, 180);
}

void Drivetrain::UpdateTelemetry()
{
  frc::SmartDashboard::PutNumber("Front Left Ang", m_frontLeft.GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Front Right Ang", m_frontRight.GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Back Left Ang", m_backLeft.GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Back Right Ang", m_backRight.GetAngle().Degrees().value());

  frc::SmartDashboard::PutNumber("Yaw", GetYaw().Degrees().value());
  frc::SmartDashboard::PutBoolean("Yaw Lock", m_YawLockActive);

  frc::SmartDashboard::PutNumber("Odometry X", m_odometry.GetPose().X().value());
}

void Drivetrain::ConfigureMotors()
{
}

void Drivetrain::Drive(frc::Trajectory::State trajectoryState, units::radian_t yaw)
{
  const auto command = m_trajectoryController.Calculate(
      m_odometry.GetPose(),
      trajectoryState,
      yaw);

  Drive(command.vx, command.vy, command.omega, false);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative)
{
  // Remember the last operating mode, for smartdash display
  m_fieldRelative = fieldRelative;

  // Heading Lock
  constexpr auto noRotThreshold = 0.1_deg_per_s;
  if (units::math::abs(rot) > noRotThreshold)
  {
    // Disable YawLock as soon as any rotation command is received
    m_YawLockActive = false;
  }
  else if (units::math::abs(rot) < noRotThreshold && units::math::abs(GetYawRate()) < 30_deg_per_s)
  {
    // Wait for the robot to stop spinning to enable Yaw Lock
    m_YawLockActive = true;
  }

  if (m_YawLockActive)
  {
    // Robot will automatically maintain current yaw
    auto r = m_yawLockPID.Calculate(GetYaw().Degrees().value());
    rot = units::degrees_per_second_t{r};
  }
  else
  {
    // Manual control, save the current yaw.
    m_yawLockPID.SetSetpoint(GetYaw().Degrees().value());
    m_yawLockPID.Reset();
  }

  // Transform Field Oriented command to a Robot Relative Command
  if (fieldRelative)
  {
    m_command = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetYaw());
  }
  else
  {
    m_command = frc::ChassisSpeeds{xSpeed, ySpeed, rot};
  }

  // Calculate desired swerve states
  auto states = m_kinematics.ToSwerveModuleStates(m_command);
  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeedLinear);

  // Set State of Each Module
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetModule(fl);
  m_frontRight.SetModule(fr);
  m_backLeft.SetModule(bl);
  m_backRight.SetModule(br);
}

void Drivetrain::Test(double y, double x)
{
  frc::SwerveModuleState f1;
  f1.speed = std::sqrt(std::pow(y, 2) + std::pow(x, 2)) * kMaxSpeedLinear;
  f1.angle = frc::Rotation2d(units::radian_t(std::atan2(-x, y)));
  m_frontLeft.SetModule(f1);
  std::cout << f1.angle.Degrees().value() << std::endl;
}

frc::Rotation2d Drivetrain::GetYaw()
{
  return frc::Rotation2d{units::degree_t{m_imu.GetAngle()}};
}

void Drivetrain::UpdateOdometry()
{
  m_odometry.Update(GetYaw(),
                    m_frontLeft.GetState(),
                    m_frontRight.GetState(),
                    m_backLeft.GetState(),
                    m_backRight.GetState());

  m_poseEstimator.Update(GetYaw(),
                         m_frontLeft.GetState(),
                         m_frontRight.GetState(),
                         m_backLeft.GetState(),
                         m_backRight.GetState());

  m_robotVelocity = m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(),
                                                  m_frontRight.GetState(),
                                                  m_backLeft.GetState(),
                                                  m_backRight.GetState()});
}

void Drivetrain::ResetYaw()
{
  m_imu.Reset();
  m_yawLockPID.SetSetpoint(GetYaw().Degrees().value());
  m_yawLockPID.Reset();
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
  m_odometry.ResetPosition(pose, GetYaw());
  m_poseEstimator.ResetPosition(pose, GetYaw());
  m_yawLockPID.SetSetpoint(GetYaw().Degrees().value());
  m_yawLockPID.Reset();
}

units::radians_per_second_t Drivetrain::GetYawRate()
{
  return units::degrees_per_second_t(m_robotVelocity.omega);
}