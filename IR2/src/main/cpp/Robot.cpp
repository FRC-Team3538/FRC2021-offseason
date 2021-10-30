// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"
#include "frc/livewindow/LiveWindow.h"

void Robot::RobotInit()
{
    // Controller Type Selection
    m_chooserControllerType.SetDefaultOption(kControllerTypePS4, frc::UniversalController::ControllerType::kPS4);
    m_chooserControllerType.AddOption(kControllerTypeXbox, frc::UniversalController::ControllerType::kXbox);
    m_chooserControllerType.AddOption(kControllerTypeStadia, frc::UniversalController::ControllerType::kStadia);
    frc::SmartDashboard::PutData("DriverType", &m_chooserControllerType);

    m_chooserOperatorType.SetDefaultOption(kControllerTypePS4, frc::UniversalController::ControllerType::kPS4);
    m_chooserOperatorType.AddOption(kControllerTypeXbox, frc::UniversalController::ControllerType::kXbox);
    m_chooserOperatorType.AddOption(kControllerTypeStadia, frc::UniversalController::ControllerType::kStadia);
    frc::SmartDashboard::PutData("OperatorType", &m_chooserOperatorType);

    // Hardware Init
    IO.ConfigureMotors();

    frc::LiveWindow::GetInstance()->SetEnabled(false);
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();

    // Subsystems Smartdash
    // frc::SmartDashboard::PutData("Driver", &m_driver);
    // frc::SmartDashboard::PutData("Operator", &m_operator);
    frc::SmartDashboard::PutData("Drivebase", &IO.drivetrain);
}

void Robot::RobotPeriodic()
{
    // Odometry
    IO.drivetrain.UpdateOdometry();

    // Drive Mode
    if (m_driver.GetOptionsButtonPressed())
        fieldCentric = !fieldCentric;

    // Gyro Reset
    if (m_driver.GetShareButtonPressed())
        IO.drivetrain.ResetYaw();

    // PS4 | xbox | Stadia controller mapping
    m_driver.SetControllerType(m_chooserControllerType.GetSelected());
    m_operator.SetControllerType(m_chooserOperatorType.GetSelected());

    // Auto Smartdash
    autoPrograms.SmartDash();

    // Subsystems Smartdash
    IO.UpdateTelemetry();

    // Vision Smartdash
    //IO.vis.Periodic();

    // robot.cpp Smartdash
    frc::SmartDashboard::PutBoolean("Field Centric", fieldCentric);
    frc::SmartDashboard::PutNumber("RPM", RPMs);
    frc::SmartDashboard::PutNumber("Target Angle", LEANGLE);
}

void Robot::AutonomousInit()
{
    IO.drivetrain.ResetYaw();
    autoPrograms.Init();
    disabledTimerOS = false;
}
void Robot::AutonomousPeriodic()
{
    autoPrograms.Run();
    IO.shooter.Periodic();
}

void Robot::TeleopInit()
{
    disabledTimerOS = false;
}
void Robot::TeleopPeriodic()
{
    // DRIVE CODE
    auto forward = -smooth_deadband(m_driver.GetY(frc::GenericHID::kLeftHand), deadbandVal, 1.0) * Drivetrain::kMaxSpeedLinear;
    auto strafe = -smooth_deadband(m_driver.GetX(frc::GenericHID::kLeftHand), deadbandVal, 1.0) * Drivetrain::kMaxSpeedLinear;
    auto rotate = -smooth_deadband(m_driver.GetX(frc::GenericHID::kRightHand), deadbandVal, 1.0) * Drivetrain::kMaxSpeedAngular * 0.75;

    //std::cout << forward << ", " << strafe << ", " << rotate << std::endl;

    IO.drivetrain.Drive(forward, strafe, rotate, fieldCentric);

    // INTAKE CODE
    double Trianglebutton = m_driver.GetTriangleButton();
    double leftTrig = smooth_deadband(m_driver.GetTriggerAxis(frc::GenericHID::kLeftHand), deadbandVal, 1.0);
    double rightTrigop = smooth_deadband(m_operator.GetTriggerAxis(frc::GenericHID::kRightHand), deadbandVal, 1.0);
    double leftTrigop = smooth_deadband(m_operator.GetTriggerAxis(frc::GenericHID::kLeftHand), deadbandVal, 1.0);
    double intakeSpd = leftTrig - Trianglebutton - leftTrigop + rightTrigop;
    IO.intake.SetSpeed(intakeSpd);

    //Deploying and Retracting I
    bool rightBump = m_driver.GetBumperPressed(frc::GenericHID::kRightHand) ;
    bool leftBump = m_driver.GetBumperPressed(frc::GenericHID::kLeftHand);
    bool rightBumpop = m_operator.GetBumperPressed(frc::GenericHID::kRightHand);
    bool leftBumpop = m_operator.GetBumperPressed(frc::GenericHID::kLeftHand);

    if (rightBump || leftBumpop || std::abs(intakeSpd) > deadbandVal)
    {
        IO.intake.SetPosition(Intake::Position::Deployed);
    }

    if (leftBump || rightBumpop || std::abs(intakeSpd) < deadbandVal)
    {
        IO.intake.SetPosition(Intake::Position::Stowed);
    }

    //Spindexer
    bool shoot = m_driver.GetTriggerAxis(frc::GenericHID::kRightHand) || m_operator.GetTriangleButton();
    double spindexer = smooth_deadband(m_operator.GetX(frc::GenericHID::kLeftHand), deadbandVal, 1.0);
    if (m_driver.GetCircleButton())
    {
        std::cout << frc::Timer::GetFPGATimestamp() << " Spindexer::Reverse" << std::endl;
        IO.spindexer.SetState(Spindexer::Reverse);
    }
    else if (shoot)
    {
        std::cout << frc::Timer::GetFPGATimestamp() << " Spindexer::Feed" << std::endl;
        IO.spindexer.SetState(Spindexer::Feed);
    }
    else if (abs(spindexer) > 0.0)
    {
        std::cout << frc::Timer::GetFPGATimestamp() << " Spindexer::Operator" << std::endl;
        IO.spindexer.Set(spindexer);
    }
    else
    {
        std::cout << frc::Timer::GetFPGATimestamp() << " Spindexer::Idle" << std::endl;
        IO.spindexer.SetState(Spindexer::Idle);
    }

    // Feeder
    if (m_driver.GetSquareButton())
    {
        IO.shooter.SetFeeder(-0.5);
    }
    else if (shoot)
    {
        IO.shooter.SetFeeder(0.5);
    }
    else
    {
        IO.shooter.SetFeeder(0.0);
    }

    // Turret
    double manualTurret = -smooth_deadband(m_operator.GetX(frc::GenericHID::kRightHand), deadbandVal, 1.0);
    IO.shooter.SetTurret(manualTurret);

    // Hood
    double manualHood = smooth_deadband(m_operator.GetY(frc::GenericHID::kRightHand), deadbandVal, 1.0);
    hoodpos += manualHood * 0.02;
    if (hoodpos > 1.0)
    {
        hoodpos = 1.0;
    }
    else if (hoodpos < 0.0)
    {
        hoodpos = 0.0;
    }

    if (m_operator.GetUpButton()) // TRIANGLE SHOT
    {
        hoodpos = 0.80; //.93
        IO.shooter.SetShooterVelocity(3250_rpm);
    }
    else if (m_operator.GetDownButton()) // GENERATOR LEG
    {
        hoodpos = 1.0;
        IO.shooter.SetShooterVelocity(3250_rpm);
    }
    else if (m_operator.GetLeftButton()) // INIT SHOT
    {
        hoodpos = 0.15;
        IO.shooter.SetShooterVelocity(3250_rpm);
    }
    else if (m_operator.GetRightButton()) // TRENCH SHOT
    {
        hoodpos = 0.16;
        IO.shooter.SetShooterVelocity(3250_rpm);
    }
    else if (m_operator.GetShareButton())
    {
        hoodpos = 1.0;
    }
    else if (m_operator.GetTouchPadButton())
        hoodpos = LEANGLE;

    IO.shooter.SetHood(hoodpos);

    frc::SmartDashboard::PutNumber("HoodPos", hoodpos);

    //Flywheel
    if (m_operator.GetCircleButton())
    {
        IO.shooter.SetShooterVelocity(units::revolutions_per_minute_t{RPMs});
    }
    else if (m_operator.GetCrossButton())
    {
        IO.shooter.SetShooterVelocity(0_rpm);
    }

    // CLIMBER CODE
    if (m_operator.GetSquareButtonPressed())
    {
        IO.climber.SetClimberPosition(Climber::State::Stowed);
    }
    else if (m_operator.GetPSButtonPressed())
    {
        IO.climber.SetClimberPosition(Climber::State::Deployed);
    }
    double telescopes = smooth_deadband(m_operator.GetY(frc::GenericHID::kLeftHand), deadbandVal, 1.0);
    IO.climber.SetClimber(telescopes);

    // JESUS
    if (m_operator.GetOptionsButtonPressed())
    {
        Jesus.Set(!Jesus.Get());
    }

    IO.shooter.Periodic();
}

void Robot::SimulationPeriodic()
{
    IO.drivetrain.SimPeriodic();
}

void Robot::DisabledInit()
{
    // Mostly for sim
    IO.drivetrain.Stop();

    IO.intake.SetPosition(Intake::Position::Stowed);
    IO.climber.SetClimberPosition(Climber::State::Deployed);
    IO.shooter.SetShooterVelocity(0_rpm);
    hoodpos = 1.0;

    if (!disabledTimerOS)
    {
        disabledTimer.Reset();
        disabledTimer.Start();
        disabledTimerOS = true;
    }

    if (disabledTimer.Get() > 5_s)
    {

    }
}

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

double Robot::smooth_deadband(double value, double deadband, double max)
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

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
