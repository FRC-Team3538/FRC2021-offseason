#include "subsystems/Drivetrain.hpp"

Drivetrain::Drivetrain()
{
#ifdef __FRC_ROBORIO__
  // m_imu.Calibrate();
#endif
  ResetYaw();

  m_yawLockPID.EnableContinuousInput(-180, 180);

  // Display Robot position on field
  frc::SmartDashboard::PutData("Field", &m_fieldDisplay);
}

void Drivetrain::UpdateTelemetry()
{
  frc::SmartDashboard::PutNumber("Front Left Ang", m_frontLeft.GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Front Right Ang", m_frontRight.GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Back Left Ang", m_backLeft.GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Back Right Ang", m_backRight.GetAngle().Degrees().value());

  frc::SmartDashboard::PutNumber("Yaw", GetYaw().Degrees().value());
  frc::SmartDashboard::PutBoolean("Yaw Lock", m_YawLockActive);

  yawLock.SetDefaultOption("Enabled", "Enabled");
  yawLock.AddOption("Disabled", "Disabled");
  if (yawLock.GetSelected() == "Enabled")
  {
    yawLockEnabled = true;
  }
  else if (yawLock.GetSelected() == "Disabled")
  {
    yawLockEnabled = false;
  }

  frc::SmartDashboard::PutData("Yaw Lock PID", &yawLock);
  std::string name = yawLock.GetSelected();
  frc::SmartDashboard::PutString("Yaw Lock PID", name);

  frc::SmartDashboard::PutNumber("Odometry X", m_odometry.GetPose().X().value());
}

void Drivetrain::ConfigureMotors()
{
  m_frontLeft.ConfigureMotors();
  m_frontRight.ConfigureMotors();
  m_backLeft.ConfigureMotors();
  m_backRight.ConfigureMotors();
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
    m_YawLockActive = yawLockEnabled; //true
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

void Drivetrain::Stop()
{
  m_frontLeft.Stop();
  m_frontRight.Stop();
  m_backLeft.Stop();
  m_backRight.Stop();
}

void Drivetrain::Test(double y, double x)
{
  frc::SwerveModuleState f1;
  f1.speed = std::sqrt(std::pow(y, 2) + std::pow(x, 2)) * kMaxSpeedLinear;
  f1.angle = frc::Rotation2d(units::radian_t(std::atan2(-x, y)));
  m_frontLeft.SetModule(f1);
  //std::cout << f1.angle.Degrees().value() << std::endl;
}

frc::Rotation2d Drivetrain::GetYaw()
{
#ifdef __FRC_ROBORIO__
  return frc::Rotation2d{units::degree_t{0}}; //m_imu.GetAngle()}};

#else
  return m_theta;
#endif
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

  auto p = m_odometry.GetPose();
  frc::Pose2d fliperoo = {-p.Y(), p.X(), p.Rotation().RotateBy(90_deg)}; // Driver Station PoV
  m_fieldDisplay.SetRobotPose(fliperoo);
}

void Drivetrain::ResetYaw()
{

#ifdef __FRC_ROBORIO__
  //m_imu.Reset();
#else
  // The ADI gyro is not simulator compatible on linux
  m_theta = 0_rad;
#endif

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

void Drivetrain::InitSendable(nt::NTSendableBuilder &builder)
{
  builder.SetSmartDashboardType("DriveBase");
  builder.SetActuator(true);

  // Modules
  m_frontLeft.InitSendable(builder, "FL");
  m_frontRight.InitSendable(builder, "FR");
  m_backLeft.InitSendable(builder, "BL");
  m_backRight.InitSendable(builder, "BR");

  // m_yawLockPID.InitSendable(builder);

  builder.AddDoubleProperty("gyro", [this] { return 0; }, nullptr); //m_imu.GetAngle(); }, nullptr);
  
  // Pose
  builder.AddDoubleProperty(
      "poseEstimator/x", [this] { return m_poseEstimator.GetEstimatedPosition().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/y", [this] { return m_poseEstimator.GetEstimatedPosition().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/yaw", [this] { return m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(); }, nullptr);

  builder.AddDoubleProperty(
      "odometry/x", [this] { return m_odometry.GetPose().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "odometry/y", [this] { return m_odometry.GetPose().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "odometry/yaw", [this] { return m_odometry.GetPose().Rotation().Degrees().value(); }, nullptr);

  // Velocity
  builder.AddDoubleProperty(
      "vel/x", [this] { return m_robotVelocity.vx.value(); }, nullptr);
  builder.AddDoubleProperty(
      "vel/y", [this] { return m_robotVelocity.vy.value(); }, nullptr);
  builder.AddDoubleProperty(
      "vel/yaw", [this] { return units::degrees_per_second_t(m_robotVelocity.omega).value(); }, nullptr);

  // Command
  builder.AddDoubleProperty(
      "cmd/x", [this] { return m_command.vx.value(); }, nullptr);
  builder.AddDoubleProperty(
      "cmd/y", [this] { return m_command.vy.value(); }, nullptr);
  builder.AddDoubleProperty(
      "cmd/yaw", [this] { return units::degrees_per_second_t(m_command.omega).value(); }, nullptr);

  // Heading Lock
  builder.AddDoubleProperty(
      "YawPID/kP", [this] { return m_yawLockPID.GetP(); }, [this](double value) { m_yawLockPID.SetP(value); });
  builder.AddDoubleProperty(
      "YawPID/kI", [this] { return m_yawLockPID.GetI(); }, [this](double value) { m_yawLockPID.SetI(value); });
  builder.AddDoubleProperty(
      "YawPID/kD", [this] { return m_yawLockPID.GetD(); }, [this](double value) { m_yawLockPID.SetD(value); });
  builder.AddDoubleProperty(
      "YawPID/SP",
      [this] { return units::degree_t(m_yawLockPID.GetSetpoint()).value(); }, nullptr);

  // Operating Mode
  builder.AddBooleanProperty(
      "cmd/fieldRelative", [this] { return m_fieldRelative; }, nullptr);
}

void Drivetrain::SimPeriodic()
{
  m_frontLeft.SimPeriodic();
  m_frontRight.SimPeriodic();
  m_backLeft.SimPeriodic();
  m_backRight.SimPeriodic();

  // Simulated IMU
#ifndef __FRC_ROBORIO__
  m_theta += m_robotVelocity.omega * 20_ms;
#endif // __FRC_ROBORIO__
}