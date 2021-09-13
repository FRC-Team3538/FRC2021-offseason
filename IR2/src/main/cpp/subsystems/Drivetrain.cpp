#include "subsystems/Drivetrain.hpp"

Drivetrain::Drivetrain()
{
}

void Drivetrain::UpdateTelemetry()
{
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
  //constexpr auto noRotThreshold = 0.1_deg_per_s;
  // if (units::math::abs(rot) > noRotThreshold)
  // {
  //   // Disable YawLock as soon as any rotation command is received
  //   m_YawLockActive = false;
  // }
  // else if (units::math::abs(rot) < noRotThreshold && units::math::abs(GetYawRate()) < 30_deg_per_s)
  // {
  //   // Wait for the robot to stop spinning to enable Yaw Lock
  //   m_YawLockActive = true;
  // }

  // if (m_YawLockActive)
  // {
  //   // Robot will automatically maintain current yaw
  //   auto r = m_yawLockPID.Calculate(GetYaw().Degrees().value());
  //   rot = units::degrees_per_second_t{r};
  // }
  // else
  // {
  //   // Manual control, save the current yaw.
  //   m_yawLockPID.SetSetpoint(GetYaw().Degrees().value());
  //   m_yawLockPID.Reset();
  // }

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
}