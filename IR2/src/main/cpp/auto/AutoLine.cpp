#include "auto/AutoLine.hpp"

// Name for Smart Dash Chooser
std::string AutoLine::GetName()
{
    return "1 - Line";
}

// Initialization
// Constructor requires a reference to the robot map
AutoLine::AutoLine(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoLine::~AutoLine() {}

//State Machine
void AutoLine::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoLine::Init()
{
    // units::feet_per_second_t maxLinearVel = 9_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    // units::feet_per_second_squared_t maxLinearAcc = 12_fps_sq;

    frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{12_mps_sq});
    config.SetReversed(false);

    std::vector<frc::Spline<5>::ControlVector> p1;

    {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Line.path");
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
void AutoLine::Run()
{
    switch (m_state)
    {
    case 0:
    {
        auto reference = m_trajectory.Sample(m_autoTimer.Get());

        IO.drivetrain.Drive(reference, 0.0_deg);

        if ((m_autoTimer.Get() > m_trajectory.TotalTime()))
        {
            NextState();
        }
        break;
    }
    default:
    {
    }
    }

    UpdateSmartDash();
}

void AutoLine::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto State", m_state);
}