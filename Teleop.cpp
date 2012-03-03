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
    shooter.InitShooter();

    driveMode = (DriveType) (int) driveChooser.GetSelected();
    controlMode = (ControlMode) (int) controlChooser.GetSelected();
    if (controlMode == kSpeed)
	EnableSpeedControl();
    else
	EnableVoltageControl();

    fireControl = kManual;

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
    DriverStationEnhancedIO *pIO = &pDS->GetEnhancedIO();
    int dsa1 = (int)(pIO->GetAnalogInRatio(1) * 2.0 + 0.5);	// 3-position switch, pickup
    int dsa2 = (int)(pIO->GetAnalogInRatio(2) * 2.0 + 0.5);	// 3-position switch, cowcatcher
    float dsa3 = pIO->GetAnalogInRatio(3);			// potentiometer, shot speed
    int dsa4 = (int)(pIO->GetAnalogInRatio(4) * 2.0 + 0.5);	// 3-position switch, shooter
    int dsa5 = (int)(pIO->GetAnalogInRatio(5) * 2.0 + 0.5);	// 3-position switch, illuminator

    bool dsd1 = pIO->GetDigital(1);	// pushbutton, fire control
    bool dsd2 = pIO->GetDigital(2);	// key switch, teach mode
    bool dsd3 = pIO->GetDigital(3);	// pushbutton, store
    bool dsd4 = pIO->GetDigital(4);	// pushbutton, target top
    bool dsd5 = pIO->GetDigital(5);	// pushbutton, target left
    bool dsd6 = pIO->GetDigital(6);	// pushbutton, target right
    bool dsd7 = pIO->GetDigital(7);	// pushbutton, target bottom
    bool dsd8 = pIO->GetDigital(8);	// pushbutton, target center

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

    if (dsd4 || dsd5 || dsd6 || dsd7 || dsd8) {
	switch (fireControl) {
	case kManual:
	    illuminator.Set(Relay::kOn);
	    target.StartAcquisition();
	    fireControl = kLooking;
	    break;

	case kLooking:
	    if (target.ProcessingComplete()) {
		if (target.TargetsFound()) {
		    Target::TargetID id = dsd4 ? Target::kTop
					: dsd5 ? Target::kLeft
					: dsd6 ? Target::kRight
					: dsd7 ? Target::kBottom
					: Target::kCenter;

		    targetLocation = target.GetTargetLocation(id);
		    // turn toward the target
		    EnablePositionControl();
		    (void) TurnToPosition(targetLocation.angle, 2.0F);
		    fireControl = kTurning;
		} else {
		    // couldn't identify target
		    fireControl = kNoTarget;
		}
	    }
	    break;

	case kTurning:
	    if (TurnToPosition(targetLocation.angle, 2.0F)) {
		// If the entire target was already visible,
		// assume we've turned the right amount and
		// fire up the shooter.  Else take another picture.
		if (targetLocation.valid) {
		    // Start the shooter.
		    // TBD: check adjustment range (dsa3) here
		    shooter.SetTarget(targetLocation.height,
				      targetLocation.distance,
				      (dsa3-0.5)*2.);
		    shooter.Start();
		    fireControl = kShooting;
		} else {
		    // More of the target should be in view now.
		    // Take another picture and reposition.
		    target.StartAcquisition();
		    fireControl = kLooking;
		}
	    }
	    break;

	case kShooting:
	    // Here's where a "ball ready" sensor would be helpful.
	    if (shooter.IsReady()) {
		shooter.Shoot();
	    }
	    shooter.Run();
	    break;

	case kNoTarget:
	    break;
	}
    } else {
	if (fireControl != kManual) {
	    // restore manual controls
	    if (controlMode == kSpeed)
		EnableSpeedControl();
	    else
		EnableVoltageControl();

	    shooter.Stop();
	}

	illuminator.Set( (dsa5 == 2) ? Relay::kOn : Relay::kOff );

	float s = 0.500 + (dsa3 * 0.400);
	shooter.SetSpeed(s);
	switch (dsa4) {
	case 2:	// up, start
	    shooter.Start();
	    break;
	case 1:	// center-off, no change
	    break;
	case 0:	// down, stop
	    shooter.Stop();
	    break;
	}
	shooter.Run();

	// This will repeat fire if the button is held down, OK?
	if (dsd1) {
	    shooter.Shoot();
	}

#if 0 // disable auto-balance code for now

	if (balance.IsBalanced()) {
	    drive.Drive(0.0F, 0.0F);
	} else {
	    if (rightTop) {
		balance.Start( false, false );
	    } else {
		balance.Stop();
#endif
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

#if 0 // disable auto-balance code

	    }
	    balance.Run();
	}
#endif

#if 0
	SmartDashboard::Log( motor_right_1.Get(), "Right1" );
	SmartDashboard::Log( motor_right_2.Get(), "Right2" );
	SmartDashboard::Log( motor_left_1.Get(),  "Left1" );
	SmartDashboard::Log( motor_left_2.Get(),  "Left2" );
#endif
    }
}

void MyRobot::TeleopContinuous()
{
    taskDelay(0);
}
