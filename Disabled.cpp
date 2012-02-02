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
    drive.StopMotor();	// safer than Drive(0.0F, 0.0F) if PWM outputs aren't properly centered
    drive.SetSafetyEnabled(false);
    
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
    ;
}

