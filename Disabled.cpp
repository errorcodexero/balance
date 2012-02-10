// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

// WPILib Includes
#include "IterativeRobot.h"

// Our Includes
#include "MyRobot.h"
#include "Version.h"

static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::DisabledInit()
{
    balance.Stop();
    DisableMotors();
    
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

