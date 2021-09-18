// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

double smooth_deadband(double value, double deadband, double max)
{
    if (std::abs(value) < deadband)
    {
        return 0.0;
    }
    else
    {
        return (value - deadband) / (max - deadband) * max;
    }
}

void Robot::RobotInit() {}
void Robot::RobotPeriodic()
{
    //IO.vis.Periodic();
    frc::SmartDashboard::PutBoolean("Field Centric", fieldCentric);
    IO.UpdateTelemetry();
    autoPrograms.SmartDash();
    IO.drivetrain.UpdateOdometry();

    if (m_driver.GetOptionsButtonPressed())
        fieldCentric = !fieldCentric;

    if (m_driver.GetShareButtonPressed())
        IO.drivetrain.ResetYaw();
}

void Robot::AutonomousInit()
{
    IO.drivetrain.ResetYaw();
    autoPrograms.Init();
}
void Robot::AutonomousPeriodic()
{
    autoPrograms.Run();
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
    // DRIVE CODE
    auto forward = -smooth_deadband(m_driver.GetY(frc::GenericHID::kLeftHand), deadbandVal, 1.0) * Drivetrain::kMaxSpeedLinear;
    auto strafe = -smooth_deadband(m_driver.GetX(frc::GenericHID::kLeftHand), deadbandVal, 1.0) * Drivetrain::kMaxSpeedLinear;
    auto rotate = -smooth_deadband(m_driver.GetX(frc::GenericHID::kRightHand), deadbandVal, 1.0) * Drivetrain::kMaxSpeedAngular;

    //std::cout << forward << ", " << strafe << ", " << rotate << std::endl;

    IO.drivetrain.Drive(forward, strafe, rotate, fieldCentric);

    // INTAKE CODE
    double rightTrig = smooth_deadband(m_driver.GetTriggerAxis(frc::GenericHID::kRightHand), deadbandVal, 1.0);
    double leftTrig = smooth_deadband(m_driver.GetTriggerAxis(frc::GenericHID::kLeftHand), deadbandVal, 1.0);

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

    // Climber

    if (m_operator.GetSquareButtonPressed())
    {
        IO.climber.SetClimberPosition(Climber::State::Deployed);
    }
    else if (m_operator.GetCircleButtonPressed())
    {
        IO.climber.SetClimberPosition(Climber::State::Stowed);
    }

    int pov = m_operator.GetPOV();

    if (pov == 0)
    {
        IO.climber.SetClimber(1.0);
    }
    else if (pov == 180)
    {
        IO.climber.SetClimber(-1.0);
    }
    else
    {
        IO.climber.SetClimber(0.0);
    }

    // SHOOTER

    if (false)
    {
        // data = IO.vis.Run();
        // if (data.filled)
        // {
        // }
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
