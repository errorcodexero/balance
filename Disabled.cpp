// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::DisabledInit()
{
    Safe();
    ShowState("Disabled", "Safe");
}

void MyRobot::DisabledPeriodic()
{
}

void MyRobot::DisabledContinuous()
{
}

