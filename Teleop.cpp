// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::TeleopInit()
{
    balance.InitBalance();
    driveMode = (DriveType) (int) driveChooser.GetSelected();
    controlMode = (ControlMode) (int) controlChooser.GetSelected();
    if (controlMode == kSpeed)
	EnableSpeedControl();
    else
	EnableVoltageControl();

    SmartDashboard::Log("Teleop", "Robot State");

    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Teleop Mode");
    lcd->UpdateLCD();
}

void MyRobot::TeleopPeriodic()
{
    if (balance.IsBalanced()) {
	drive.Drive(0.0F, 0.0F);
    } else {
	float leftY  = joy_left.GetY();
	float rightY = joy_right.GetY();
	float rightX = - joy_right.GetX();
	float rightT = - joy_right.GetTwist();
	bool rightTrigger = joy_right.GetTrigger();
	bool rightTop     = joy_right.GetTop();

	if (rightTop) {
	    balance.Start( false, false );
	    balance.Run();
	} else {
	    balance.Stop();
	    switch (driveMode) {
	    case kFlightStick:
		    drive.ArcadeDrive( rightY, rightT, !rightTrigger );
		    break;
	    case kArcade:
		    drive.ArcadeDrive( rightY, rightX, !rightTrigger );
		    break;
	    case kXY:
		    if (rightY > 0.05) {
			drive.ArcadeDrive( rightY, -rightX, !rightTrigger );
		    } else {
			drive.ArcadeDrive( rightY, rightX, !rightTrigger );
		    }
		    break;
	    case kTwoStick:
		    drive.TankDrive( rightY, leftY );
		    break;
	    }
	}
    }

    SmartDashboard::Log( motor_right_1.Get(), "Right1" );
    SmartDashboard::Log( motor_right_2.Get(), "Right2" );
    SmartDashboard::Log( motor_left_1.Get(),  "Left1" );
    SmartDashboard::Log( motor_left_2.Get(),  "Left2" );
}

void MyRobot::TeleopContinuous()
{
    taskDelay(0);
}
