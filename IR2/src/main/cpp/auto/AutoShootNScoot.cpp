#include "auto/AutoShootNScoot.hpp"

#include <frc/Filesystem.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

// Name for Smart Dash Chooser
std::string AutoShootNScoot::GetName()
{
    return "4 - ShootNScoot";
}

// Initialization
// Constructor requires a reference to the robot map
AutoShootNScoot::AutoShootNScoot(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoShootNScoot::~AutoShootNScoot() {}

//State Machine
void AutoShootNScoot::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    shootOS = false;
}

void AutoShootNScoot::Init()
{
    units::feet_per_second_t maxLinearVel = 15_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 15_fps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    //config.AddConstraint(frc::CentripetalAccelerationConstraint{12_mps_sq});
    config.SetReversed(false);

    std::vector<frc::Spline<5>::ControlVector> p1;

    {
        wpi::SmallString<256> filePath;
        frc::filesystem::GetDeployDirectory(filePath);
        wpi::sys::path::append(filePath, "PathWeaver");
        wpi::sys::path::append(filePath, "Paths");
        wpi::sys::path::append(filePath, "Line.path");

        io::CSVReader<6> csv(filePath.c_str());
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
            //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
            p1.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
    }

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(p1, config);

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(m_trajectory.InitialPose());
}

// Execute the program
void AutoShootNScoot::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.climber.SetClimberPosition(Climber::State::Stowed);
        IO.intake.SetPosition(Intake::Position::Deployed);

        auto targetVel = 3250_rpm;
        double hoodPreset = 0.15;

        auto spinupTime = 0.0_s;

        IO.shooter.SetHood(hoodPreset);

        if (IO.shooter.SetShooterVelocity(targetVel))
        {
            if (!shootOS)
                spinupTime = m_autoTimer.Get();
            shootOS = true;

            if (shootOS && (m_autoTimer.Get() > (spinupTime + 1_s)))
            {
                IO.shooter.SetFeeder(0.6);
                IO.spindexer.SetState(Spindexer::State::Feed);
            }
        }
        else
        {
            IO.shooter.SetFeeder(0.0);
            IO.spindexer.SetState(Spindexer::State::Idle);
        }

        if (m_autoTimer.Get() > 7.0_s)
            NextState();

        break;
    }
    case 1:
    {
        IO.shooter.SetShooterVelocity(0.0_rpm);
        IO.shooter.SetFeeder(0.0);
        IO.spindexer.SetState(Spindexer::State::Idle);

        auto theta = 0.0_deg;
        auto thetaPrime = 180.0_deg;

        auto reference = m_trajectory.Sample(m_autoTimer.Get());

        auto yaw = theta + ((thetaPrime - theta) / (m_trajectory.TotalTime() * 0.75) * m_autoTimer.Get());
        yaw = units::math::abs(yaw - theta) > units::math::abs(thetaPrime - theta) ? thetaPrime : yaw;

        IO.drivetrain.Drive(reference, 0.0_rad);

        if ((m_autoTimer.Get() > m_trajectory.TotalTime()))
        {
            NextState();
        }
        break;
    }
    default:
    {
        IO.drivetrain.Stop();
    }
    }

    UpdateSmartDash();
}

void AutoShootNScoot::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto State", m_state);
}