#include "subsystems/SwerveModule.hpp"

/**
 * Constructor for the SwerveModule class
 * 
 * @param moduleID String for module identification
 * @param driveMotorChannel CAN ID of the drive Falcon
 * @param turningMotorChannel CAN ID of the turning Falcon
 * @param turningEncoderChannel CAN ID of turning ABS encoder
 * @param config The configuration data for this specific module
 */
SwerveModule::SwerveModule(std::string moduleID, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, SwerveModuleConfig config)
    : moduleID(moduleID),
      m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      turningEncAbs(turningEncoderChannel),
      m_drivePIDController{config.drivePID.kP, config.drivePID.kI, config.drivePID.kD, {config.drivePID.max_acceleration, config.drivePID.max_jerk}},
      m_turningPIDController{config.turningPID.kP, config.turningPID.kI, config.turningPID.kD, {config.turningPID.max_angular_velocity, config.turningPID.max_angular_acceleration}},
      m_driveFeedforward{config.driveFf.kS, config.driveFf.kV, config.driveFf.kA},
      m_turnFeedforward{config.turnFf.kS, config.turnFf.kV, config.turnFf.kA}
{
  // Drive Motor Configuration
  m_driveMotor.ConfigFactoryDefault();
  m_driveMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
  m_driveMotor.SetInverted(true); // Remember: forward-positive!
  m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_driveMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, kDriveMotorCurrentLimit.value(), kDriveMotorCurrentLimit.value(), 0.0));
  m_driveMotor.SetSensorPhase(false);

  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
  // m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);

  // Turning Motor Configuration
  m_turningMotor.ConfigFactoryDefault();
  m_turningMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
  m_turningMotor.SetInverted(false); // Remember: forward-positive!
  m_turningMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_turningMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, kTurningMotorCurrentLimit.value(), kTurningMotorCurrentLimit.value(), 0.0));

  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);


  // Turning Encoder Config
  ctre::phoenix::sensors::CANCoderConfiguration encoderConfig;
  turningEncAbs.GetAllConfigs(encoderConfig);
  encoderConfig.enableOptimizations = true;
  encoderConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
  encoderConfig.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
  encoderConfig.magnetOffsetDegrees = config.angleOffset.value();
  encoderConfig.sensorDirection = false;
  turningEncAbs.ConfigAllSettings(encoderConfig);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t(wpi::numbers::pi),
                                               units::radian_t(wpi::numbers::pi));
  m_turningPIDController.Reset(units::degree_t(turningEncAbs.GetAbsolutePosition()));
}

/**
 * Returns the current state of the module
 * 
 * @return frc::SwerveModuleState The current state of the swerve module
 */
frc::SwerveModuleState SwerveModule::GetState()
{
  currentState.angle = GetAngle();
  currentState.speed = GetVelocity();
  return currentState;
}

/**
 * Returns the current velocity of the module
 * 
 * @return units::meters_per_second_t The current velocity of the swerve module
 */
units::meters_per_second_t SwerveModule::GetVelocity()
{
#ifdef __FRC_ROBORIO__
  // Real Hardware
  return m_driveMotor.GetSelectedSensorVelocity(0) * kDriveScaleFactor / 100_ms;
#else
  // Simulation
  return m_driveSim.GetVelocity();
#endif
}

/**
 * Returns the current angle of the module
 * 
 * @return frc::Rotation2d The current angle of the swerve module
 */
frc::Rotation2d SwerveModule::GetAngle()
{
#ifdef __FRC_ROBORIO__
  // Real Hardware
  auto un_normalized = frc::Rotation2d(units::degree_t(turningEncAbs.GetAbsolutePosition()));
  return frc::Rotation2d(un_normalized.Cos(), un_normalized.Sin());
#else
  // Simulation
  auto un_normalized = frc::Rotation2d(units::degree_t(m_turnSim.GetPosition()));
  return frc::Rotation2d(un_normalized.Cos(), un_normalized.Sin());
#endif
}

/**
 * Sets the module's target state
 * 
 * @param state The module's target state
 */
void SwerveModule::SetModule(const frc::SwerveModuleState &state)
{
  currentState = GetState();

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
  //std::cout << m_driveVolts.value() << std::endl;
  m_driveMotor.SetVoltage(m_driveVolts);
  m_turningMotor.SetVoltage(m_turnVolts);

  // Simulation
#ifndef __FRC_ROBORIO__
  m_driveSim.SetInputVoltage(m_driveVolts);
  m_turnSim.SetInputVoltage(m_turnVolts);
#endif
}

/**
 * Just Stop....
 *
 */
void SwerveModule::Stop()
{
  m_drivePIDController.Reset(0_mps);
  m_turningPIDController.Reset(0_deg);

  m_driveMotor.SetVoltage(0_V);
  m_turningMotor.SetVoltage(0_V);

  // Simulation
#ifndef __FRC_ROBORIO__
  m_driveSim.SetInputVoltage(0_V);
  m_turnSim.SetInputVoltage(0_V);
#endif
}

void SwerveModule::ConfigureMotors()
{
}

void SwerveModule::UpdateTelemetry()
{
}

void SwerveModule::InitSendable(nt::NTSendableBuilder &builder)
{
  InitSendable(builder, "");
}

void SwerveModule::InitSendable(nt::NTSendableBuilder &builder, std::string name)
{
  builder.SetSmartDashboardType("SwerveModule");
  builder.SetActuator(true);

  // Prefix for nested objects
  if (name != "")
    name += "/";

  // Drive Control
  builder.AddDoubleProperty(
      name + "Drive kP", [this] { return m_drivePIDController.GetP(); }, [this](double value) { m_drivePIDController.SetP(value); });
  builder.AddDoubleProperty(
      name + "Drive kI", [this] { return m_drivePIDController.GetI(); }, [this](double value) { m_drivePIDController.SetI(value); });
  builder.AddDoubleProperty(
      name + "Drive kD", [this] { return m_drivePIDController.GetD(); }, [this](double value) { m_drivePIDController.SetD(value); });
  builder.AddDoubleProperty(
      name + "Drive Goal",
      [this] { return units::meters_per_second_t(m_drivePIDController.GetGoal().position).value(); },
      [this](double value) { m_drivePIDController.SetGoal(units::meters_per_second_t(value)); });
  builder.AddDoubleProperty(
      name + "Drive SP",
      [this] { return units::meters_per_second_t(m_drivePIDController.GetSetpoint().position).value(); }, nullptr);
  builder.AddDoubleProperty(
      name + "Velocity", [this] { return units::meters_per_second_t(GetVelocity()).value(); }, nullptr);
  builder.AddDoubleProperty(
      name + "m_driveVolts", [this] { return m_driveVolts.value(); }, nullptr);

  // Angle Control
  builder.AddDoubleProperty(
      name + "Angle kP", [this] { return m_turningPIDController.GetP(); }, [this](double value) { m_turningPIDController.SetP(value); });
  builder.AddDoubleProperty(
      name + "Angle kI", [this] { return m_turningPIDController.GetI(); }, [this](double value) { m_turningPIDController.SetI(value); });
  builder.AddDoubleProperty(
      name + "Angle kD", [this] { return m_turningPIDController.GetD(); }, [this](double value) { m_turningPIDController.SetD(value); });
  builder.AddDoubleProperty(
      name + "Angle Goal",
      [this] { return units::degree_t(m_turningPIDController.GetGoal().position).value(); },
      [this](double value) { m_turningPIDController.SetGoal(units::degree_t(value)); });
  builder.AddDoubleProperty(
      name + "Angle SP",
      [this] { return units::degree_t(m_turningPIDController.GetSetpoint().position).value(); }, nullptr);
  builder.AddDoubleProperty(
      name + "Angle", [this] { return GetAngle().Degrees().value(); }, nullptr);
  builder.AddDoubleProperty(
      name + "m_turnVolts", [this] { return m_turnVolts.value(); }, nullptr);

  // builder.AddDoubleProperty(
  //     "Angle Offset",
  //     [this] { return prefs->GetDouble(m_angleOffsetPref); },
  //     [this](double value) {
  //         prefs->PutDouble(m_angleOffsetPref, value);
  //         m_turningEncoder.ConfigMagnetOffset(value);
  //     });

  // Turning Encoders
  // builder.AddDoubleProperty(
  //     "Encoder CTRE", [this] { return m_turningEncoder.GetAbsolutePosition(); }, nullptr);

  // Thermal
  builder.AddDoubleProperty(
      name + "Drive Temp [C]", [this] { return m_driveMotor.GetTemperature(); }, nullptr);
  builder.AddDoubleProperty(
      name + "Angle Temp [C]", [this] { return m_turningMotor.GetTemperature(); }, nullptr);
}

void SwerveModule::SimPeriodic()
{
  m_driveSim.Update(20_ms);
  m_turnSim.Update(20_ms);
}