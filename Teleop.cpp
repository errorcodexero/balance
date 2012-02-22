// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::TeleopInit()
{
    Safe();
    compressor.Start();
    lastPickup = 0;

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
    float leftY  = joy_left.GetY();
//  float leftX  = joy_left.GetX();
//  float leftT  = joy_left.GetTwist();
//  bool leftTrigger = joy_left.GetTrigger();
//  bool leftTop     = joy_left.GetTop();

    float rightY = joy_right.GetY();
    float rightX = joy_right.GetX();
    float rightT = joy_right.GetTwist();
    bool rightTrigger = joy_right.GetTrigger();
    bool rightTop     = joy_right.GetTop();

    DriverStation *pDS = DriverStation::GetInstance();
    int dsa1 = (int)((pDS->GetAnalogIn(1) * 2.0 / 3.3) + 0.5);    // 3-position switch, pickup
    int dsa2 = (int)((pDS->GetAnalogIn(2) * 2.0 / 3.3) + 0.5);    // 3-position switch, cowcatcher
    int dsa3 = (int)((pDS->GetAnalogIn(3) * 256.0 / 3.3) + 0.5);  // potentiometer, shot speed?
    bool dsd1 = pDS->GetDigitalIn(1);	// pushbutton, fire control
    bool dsd2 = pDS->GetDigitalIn(2);	// key switch, shooter motor enable
    bool dsd3 = pDS->GetDigitalIn(3);	// pushbotton, store

    if (dsa1 != lastPickup) {
	int d = pickup.GetDirection();
	switch (dsa1) {
	case 0:	// down, change forward->stopped, stopped->reverse
	    if (d >= 0) --d;
	    break;
	case 1:	// center-off, no change
	    break;
	case 2:	// up, change reverse->stopped, stopped->forward
	    if (d <= 0) ++d;
	    break;
	}
	pickup.SetDirection(d);
	lastPickup = dsa1;
    }

    switch (dsa2) {
    case 0:
	cowcatcher.Set( true );
	break;
    case 1:
	break;
    case 2:
	cowcatcher.Set( false );
	break;
    }

    if (dsd1) {
	ball_injector.Set( true );
    } else {
	ball_injector.Set( false );
    }

    if (dsd2) {
	shooter.Start();
    } else {
	shooter.Stop();
    }
    shooter.Run();

    if (balance.IsBalanced()) {
	drive.Drive(0.0F, 0.0F);
    } else {
	if (rightTop) {
	    balance.Start( false, false );
	} else {
	    balance.Stop();
	    switch (driveMode) {
	    case kFlightStick:
		    drive.ArcadeDrive( rightY, -rightT, !rightTrigger );
		    break;
	    case kArcade:
		    drive.ArcadeDrive( rightY, -rightX, !rightTrigger );
		    break;
	    case kXY:
		    if (rightY > 0.10) {
			drive.ArcadeDrive( rightY, rightX, !rightTrigger );
		    } else {
			drive.ArcadeDrive( rightY, -rightX, !rightTrigger );
		    }
		    break;
	    case kTwoStick:
		    drive.TankDrive( rightY, leftY );
		    break;
	    }
	}
	balance.Run();
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
