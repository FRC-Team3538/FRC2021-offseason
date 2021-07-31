#include "subsystems/SwerveModule.hpp"

SwerveModule::SwerveModule(std::string moduleID, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      turningEncAbs(turningEncoderChannel)
{
}

frc::SwerveModuleState SwerveModule::GetState()
{
  return currentState;
}

void SwerveModule::SetModule(const frc::SwerveModuleState &state)
{
  targetState = state;
  
  frc::SwerveModuleState::Optimize(targetState, currentState.angle);

  // Drive
  
}