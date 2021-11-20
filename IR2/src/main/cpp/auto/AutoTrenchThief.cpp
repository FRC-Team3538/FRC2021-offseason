#include "auto/AutoTrenchThief.hpp"

#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <wpi/SmallString.h>

// Name for Smart Dash Chooser
std::string AutoTrenchThief::GetName()
{
    return "3 - Trench Thief";
}

// Initialization
// Constructor requires a reference to the robot map
AutoTrenchThief::AutoTrenchThief(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoTrenchThief::~AutoTrenchThief() {}

//State Machine
void AutoTrenchThief::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoTrenchThief::Init()
{
    units::feet_per_second_t maxLinearVel = 4_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 4_fps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    //config.AddConstraint(frc::CentripetalAccelerationConstraint{12_mps_sq});
    config.SetReversed(false);

    std::vector<frc::Spline<5>::ControlVector> p1;

    {
        std::string filePath = frc::filesystem::GetDeployDirectory();
        filePath = fs::path{filePath}.append("PathWeaver").append("Paths").append("Thief1.path").c_str();
        

        io::CSVReader<6> csv(filePath);
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
            //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
            p1.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
    }

    m_trajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(p1, config);

    std::vector<frc::Spline<5>::ControlVector> p2;

    {        
        std::string filePath = frc::filesystem::GetDeployDirectory();
        filePath = fs::path{filePath}.append("PathWeaver").append("Paths").append("Thief2.path").c_str();
        

        io::CSVReader<6> csv(filePath);
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
            //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
            p2.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
    }

    m_trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(p2, config);

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(m_trajectory1.InitialPose());
}

// Execute the program
void AutoTrenchThief::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.intake.SetPosition(Intake::Position::Deployed);
        IO.intake.SetSpeed(1);
        NextState();
        break;
    }
    case 1:
    {
        auto reference = m_trajectory1.Sample(m_autoTimer.Get());

        IO.drivetrain.Drive(reference, 0.0_deg);

        if ((m_autoTimer.Get() > m_trajectory1.TotalTime()))
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        auto reference = m_trajectory2.Sample(m_autoTimer.Get());

        IO.drivetrain.Drive(reference, 0.0_deg);

        if ((m_autoTimer.Get() > m_trajectory2.TotalTime()))
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.intake.SetPosition(Intake::Position::Stowed);
        IO.intake.SetSpeed(0);
        NextState();
        break; 
    }
    default:
    {
        IO.drivetrain.Stop();
    }
    }

    UpdateSmartDash();
}

void AutoTrenchThief::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto State", m_state);
}