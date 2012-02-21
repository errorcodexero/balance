// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::DisabledInit()
{
    Safe();

    SmartDashboard::Log("Disabled", "Robot State");

    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Disabled");
    lcd->UpdateLCD();
}

void MyRobot::DisabledPeriodic()
{
    ;
}

void MyRobot::DisabledContinuous()
{
    taskDelay(0);
}

