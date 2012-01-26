// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

// WPILib Includes
#include "IterativeRobot.h"

// Our Includes
#include "MyRobot.h"
#include "Version.h"

static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::TeleopInit()
{
    balance.Stop();
    drive.StopMotor();
    
    SmartDashboard::Log("Teleop", "Robot State");

    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Teleop Mode");
    lcd->UpdateLCD();

    driveMode = (DriveType) (int) driveChooser.GetSelected();
}

void MyRobot::TeleopPeriodic()
{
    float rightX = joy_right.GetX();
    float rightY = joy_right.GetY();
    float rightT = joy_right.GetTwist();
    bool rightTrigger = joy_right.GetTrigger();
    bool rightTop = joy_right.GetTop();
    float leftY = joy_left.GetY();

    if (rightTop) {
	balance.Start( 0.5F, false );
    } else {
	balance.Stop();
	if (rightTrigger) {
	    rightX /= 3.0;
	    rightY /= 3.0;
	    leftY /= 3.0;
	}

	switch (driveMode) {
	case kFlightStick:
	    drive.ArcadeDrive( rightY, rightT, true );
	    break;
	case kArcade:
	    drive.ArcadeDrive( rightY, rightX, true );
	    break;
	case kTwoStick:
	    drive.TankDrive( rightY, leftY );
	    break;
	}
    }
}

void MyRobot::TeleopContinuous()
{
    balance.Run();
}
