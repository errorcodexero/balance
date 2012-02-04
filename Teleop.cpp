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
    balance.InitBalance();
    drive.Drive( 0.0F, 0.0F );
    drive.SetSafetyEnabled(true);
    
    SmartDashboard::Log("Teleop", "Robot State");

    driveMode = (DriveType) (int) driveChooser.GetSelected();

    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Teleop Mode");
    lcd->UpdateLCD();
}

void MyRobot::TeleopPeriodic()
{
    if (balance.IsBalanced()) {
	drive.Drive(0.0F, 0.0F);
    } else {
	float rightX = -joy_right.GetX();
	float rightY = -joy_right.GetY();
	float rightT = -joy_right.GetTwist();
	bool rightTrigger = joy_right.GetTrigger();
	bool rightTop = joy_right.GetTop();
	float leftY = -joy_left.GetY();

	if (rightTop) {
	    balance.Start( false, false );
	    balance.Run();
	} else {
	    balance.Stop();
	    switch (driveMode) {
	    case kFlightStick:
		    drive.ArcadeDrive( rightY, rightT / 2.0F, !rightTrigger );
		    break;
	    case kArcade:
		    drive.ArcadeDrive( rightY, rightX / 2.0F, !rightTrigger );
		    break;
	    case kTwoStick:
		    drive.TankDrive( rightY, leftY );
		    break;
	    }
	}
    }
}

void MyRobot::TeleopContinuous()
{
    taskDelay(0);
}
