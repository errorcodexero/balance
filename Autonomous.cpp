// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "MyRobot.h"
#include "AutoCommand.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::AutonomousInit()
{
    Safe();
    compressor.Start();
    ShowState("Autonomous","");
    m_autoCommand.Start();
}

void MyRobot::AutonomousPeriodic()
{
    m_autoCommand.Run();
}

void MyRobot::AutonomousContinuous()
{
}

