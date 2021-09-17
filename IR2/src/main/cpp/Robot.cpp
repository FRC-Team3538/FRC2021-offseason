// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit() {}
void Robot::RobotPeriodic()
{
  //IO.vis.Periodic();
  IO.UpdateTelemetry();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
  // DRIVE CODE
  auto forward = Deadband(m_driver.GetY(frc::GenericHID::kLeftHand), deadbandVal) * Drivetrain::kMaxSpeedLinear;
  auto strafe = -Deadband(m_driver.GetX(frc::GenericHID::kLeftHand), deadbandVal) * Drivetrain::kMaxSpeedLinear;
  auto rotate = -Deadband(m_driver.GetX(frc::GenericHID::kRightHand), deadbandVal) * Drivetrain::kMaxSpeedAngular;

  IO.drivetrain.Drive(forward, strafe, rotate, true);

  // INTAKE CODE
  double rightTrig = Deadband(m_driver.GetTriggerAxis(frc::GenericHID::kRightHand), deadbandVal);
  double leftTrig = Deadband(m_driver.GetTriggerAxis(frc::GenericHID::kLeftHand), deadbandVal);

  bool rightBump = m_driver.GetBumperPressed(frc::GenericHID::kRightHand);
  bool leftBump = m_driver.GetBumperPressed(frc::GenericHID::kLeftHand);

  if (rightBump)
  {
    IO.intake.SetPosition(Intake::Position::Deployed);
  }

  if (leftBump)
  {
    IO.intake.SetPosition(Intake::Position::Stowed);
  }

  IO.intake.SetSpeed(-leftTrig + rightTrig);

  // SHOOTER

  if (false)
  {
    data = IO.vis.Run();
    if(data.filled)
    {
      
    }
  }
  else
  {
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

double Robot::Deadband(double value, double deadband)
{
  if ((std::abs(value)) < deadband)
  {
    return 0.0;
  }
  else if (value > 0.95)
  {
    return 1.0;
  }
  else if (value < -0.95)
  {
    return -1.0;
  }
  else
  {
    return value;
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
