#include "auto/AutoTest.hpp"

#include <frc/Filesystem.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

// Name for Smart Dash Chooser
std::string AutoTest::GetName()
{
    return "2 - Test";
}

// Initialization
// Constructor requires a reference to the robot map
AutoTest::AutoTest(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoTest::~AutoTest() {}

//State Machine
void AutoTest::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoTest::Init()
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
        wpi::sys::path::append(filePath, "Curve.path");

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
void AutoTest::Run()
{
    switch (m_state)
    {
    case 0:
    {
        auto theta = 0.0_deg;
        auto thetaPrime = 180.0_deg;

        auto reference = m_trajectory.Sample(m_autoTimer.Get());

        auto yaw = theta + ((thetaPrime - theta)/(m_trajectory.TotalTime() * 0.75) * m_autoTimer.Get());
        yaw = units::math::abs(yaw - theta) > units::math::abs(thetaPrime - theta) ? thetaPrime : yaw;

        IO.drivetrain.Drive(reference, yaw);

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

void AutoTest::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto State", m_state);
}