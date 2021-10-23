#include "auto/AutoPrograms.hpp"

// Include all auto programs [List 1 of 3]
#include "auto/AutoLine.hpp"
#include "auto/AutoTrenchThief.hpp"
#include "auto/AutoTest.hpp"
#include "auto/AutoShootNScoot.hpp"

// Constructor requires a reference to the robot map
AutoPrograms::AutoPrograms(Robotmap &IO) : IO(IO)
{
    // SmartDash Chooser [List 2 of 3]
    m_chooser.SetDefaultOption("0 - None", "0 - None");
    m_chooser.AddOption(AutoLine::GetName(), AutoLine::GetName());
    m_chooser.AddOption(AutoTest::GetName(), AutoTest::GetName());
    m_chooser.AddOption(AutoTrenchThief::GetName(), AutoTrenchThief::GetName());
    m_chooser.AddOption(AutoShootNScoot::GetName(), AutoShootNScoot::GetName());
}

// Initialize the selected auto program
void AutoPrograms::Init()
{
    // Get Selected Program from SmartDash Chooser
    std::string name = m_chooser.GetSelected();

    // Delete previously selected auto program
    delete m_autoProgram;
    m_autoProgram = NULL;

    // Create the Selected auto program [List 3 of 3]
    if (name == AutoLine::GetName())
    {
        m_autoProgram = new AutoLine(IO);
    }
    if (name == AutoTest::GetName())
    {
        m_autoProgram = new AutoTest(IO);
    }
    if (name == AutoTrenchThief::GetName())
    {
        m_autoProgram = new AutoTrenchThief(IO);
    }
    if(name == AutoShootNScoot::GetName())
    {
        m_autoProgram = new AutoShootNScoot(IO);
    }
    if (m_autoProgram != NULL)
        m_autoProgram->Init();
}

// Run the selected Auto Program
void AutoPrograms::Run()
{
    if (m_autoProgram != NULL)
    {
        m_autoProgram->Run();
    }
}

void AutoPrograms::SmartDash()
{
    frc::SmartDashboard::PutData("Choose Auto", &m_chooser);
    std::string name = m_chooser.GetSelected();
    frc::SmartDashboard::PutString("Selected Auto", name);
}