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
    int dsa1 = (int)((pDS->GetAnalogIn(1) * 2.0 / 3.3) + 0.5);	// 3-position switch, pickup
    int dsa2 = (int)((pDS->GetAnalogIn(2) * 2.0 / 3.3) + 0.5);	// 3-position switch, cowcatcher
    float dsa3 = (pDS->GetAnalogIn(3) * 1200.0 / 3.3);		// potentiometer, shot speed?
    int dsa4 = (int)((pDS->GetAnalogIn(4) * 2.0 / 3.3) + 0.5);	// 3-position switch, shooter

// dsa5 isn't usable: WPILib doesn't allow analog channels 5-8
// even though they show up on the driver station
//  int dsa5 = (int)((pDS->GetAnalogIn(5) * 2.0 / 3.3) + 0.5);	// 3-position switch, target height

    bool dsd1 = pDS->GetDigitalIn(1);	// pushbutton, fire control
    bool dsd2 = pDS->GetDigitalIn(2);	// key switch, teach mode
    bool dsd3 = pDS->GetDigitalIn(3);	// pushbotton, store

    switch (dsa1) {
    case 2:	// up, forward
	pickup.Forward();
	break;
    case 1:	// center, off
        pickup.Stop();
	break;
    case 0:	// down, reverse
	pickup.Reverse();
	break;
    }

    switch (dsa2) {
    case 2:
	cowcatcher.Set( false );
	break;
    case 1:
	break;
    case 0:
	cowcatcher.Set( true );
	break;
    }

    switch (dsa4) {
    case 2:	// up, start
	shooter.Start(dsa3);
	break;
    case 1:	// center-off, no change
	shooter.Run();
	break;
    case 0:	// down, stop
	shooter.Stop();
	break;
    }

    if (dsd1) {
	ball_injector.Set( true );
    } else {
	ball_injector.Set( false );
    }


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
