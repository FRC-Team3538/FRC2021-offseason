// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define RPMs (frc::SmartDashboard::GetNumber("RPM", 2750.0))

#include <frc/TimedRobot.h>
#include "lib/UniversalController.hpp"
#include "Robotmap.hpp"
#include "auto/AutoPrograms.hpp"
#include <frc/Solenoid.h>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationPeriodic() override;

private:

  frc::SendableChooser<frc::UniversalController::ControllerType> m_chooserControllerType;
  frc::SendableChooser<frc::UniversalController::ControllerType> m_chooserOperatorType;
  static constexpr auto kControllerTypePS4 = "PS4";
  static constexpr auto kControllerTypeXbox = "Xbox";
  static constexpr auto kControllerTypeStadia = "Stadia";

  frc::UniversalController m_driver{0};
  frc::UniversalController m_operator{1};

  Robotmap IO;

  double Deadband(double value, double deadband);
  double smooth_deadband(double value, double deadband, double max);

  const double deadbandVal = 0.1;

  bool fieldCentric = true;

  vision::RJVisionPipeline::visionData data;
  double distance = -1.0;

  AutoPrograms autoPrograms{IO};

  bool shooterLocked = false;

  frc::Solenoid Jesus{7};
};
