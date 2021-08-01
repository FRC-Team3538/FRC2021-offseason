#include "subsystems/SwerveModule.hpp"

SwerveModule::SwerveModule(std::string moduleID, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, SwerveModuleConfig config)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      turningEncAbs(turningEncoderChannel),
      m_drivePIDController{config.drivePID.kP, config.drivePID.kI, config.drivePID.kD, {config.drivePID.max_acceleration, config.drivePID.max_jerk}},
      m_turningPIDController{config.turningPID.kP, config.turningPID.kI, config.turningPID.kD, {config.turningPID.max_angular_velocity, config.turningPID.max_angular_acceleration}},
      m_driveFeedforward{config.driveFf.kS, config.driveFf.kV, config.driveFf.kA},
      m_turnFeedforward{config.turnFf.kS, config.turnFf.kV, config.turnFf.kA}
{
}

frc::SwerveModuleState SwerveModule::GetState()
{
  return currentState;
}

units::meters_per_second_t SwerveModule::GetVelocity()
{
  return m_driveMotor.GetSelectedSensorVelocity(0) * kDriveScaleFactor / 100_ms;
}

frc::Rotation2d SwerveModule::GetAngle()
{
  return frc::Rotation2d(units::degree_t(turningEncAbs.Get()));
}

void SwerveModule::SetModule(const frc::SwerveModuleState &state)
{
  targetState = state;

  const auto opt_state = frc::SwerveModuleState::Optimize(targetState, currentState.angle);

  // Drive
  const auto driveOutput = m_drivePIDController.Calculate(
      currentState.speed,
      opt_state.speed);

  const auto driveFeedforward = m_driveFeedforward.Calculate(m_drivePIDController.GetSetpoint().position, m_drivePIDController.GetSetpoint().velocity);

  const auto m_driveVolts = units::volt_t{driveOutput} + driveFeedforward;

  // Angle
  const auto turnOutput = m_turningPIDController.Calculate(
      currentState.angle.Radians(),
      opt_state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  const auto m_turnVolts = units::volt_t{turnOutput} + turnFeedforward;

  // Output
  m_driveMotor.SetVoltage(m_driveVolts);
  m_turningMotor.SetVoltage(m_turnVolts);
}

