// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::DisabledInit()
{
    Safe();
    m_driveCommand.Stop();
    m_turnCommand.Stop();
    m_shootCommand.Stop();
    m_autoCommand.Stop();
    compressor.Start();
    ShowState("Disabled", "Safe");
}

void MyRobot::DisabledPeriodic()
{
    taskDelay(1);
}

void MyRobot::DisabledContinuous()
{
    taskDelay(1);
}

