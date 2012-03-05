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
    int dsa5 = (int)(pIO->GetAnalogInRatio(5) * 2.0 + 0.5);	// 3-position switch, unused

    bool dsd1 = pIO->GetDigital(1);	// pushbutton, fire control
    bool dsd2 = pIO->GetDigital(2);	// key switch, teach mode
    bool dsd3 = pIO->GetDigital(3);	// pushbutton, store
    bool dsd4 = pIO->GetDigital(4);	// pushbutton, target top
    bool dsd5 = pIO->GetDigital(5);	// pushbutton, target left
    bool dsd6 = pIO->GetDigital(6);	// pushbutton, target right
    bool dsd7 = pIO->GetDigital(7);	// pushbutton, target bottom
    bool dsd8 = pIO->GetDigital(8);	// toggle switch, illuminator

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

#if 1
    cowcatcher.Set( rightTrigger );
#else
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
#endif

    if (dsd4 || dsd5 || dsd6 || dsd7) {
	switch (fireControl) {
	case kManual:
	    printf("Starting targeting sequence! %d %d %d %d\n", dsd4, dsd5, dsd6, dsd7);
	    illuminator.Set(Relay::kOn);
	    DisableMotors();
	    target.StartAcquisition();
	    fireControl = kLooking;
	    break;

	case kLooking:
	    if (target.ProcessingComplete()) {
		printf("Target processing complete\n");
		if (target.TargetsFound()) {
		    printf("Targets found!\n");
		    Target::TargetID id = dsd4 ? Target::kTop
					: dsd5 ? Target::kLeft
					: dsd6 ? Target::kRight
					: dsd7 ? Target::kBottom
					: Target::kCenter;  // can't happen

		    targetLocation = target.GetTargetLocation(id);
		    // turn toward the target
		    EnablePositionControl();
		    printf("Starting turn: %g degrees\n", targetLocation.angle);
		    (void) TurnToPosition(targetLocation.angle, 0.50);
		    fireControl = kTurning;
		} else {
		    // couldn't identify target
		    printf("No targets visible\n");
		    fireControl = kNoTarget;
		}
	    }
	    break;

	case kTurning:
	    if (TurnToPosition(targetLocation.angle, 0.50)) {
		printf("Turn to target complete!\n");
		// If the entire target was already visible,
		// assume we've turned the right amount and
		// fire up the shooter.  Else take another picture.
		if (targetLocation.valid) {
		    printf("Starting shooter, distance = %g\n", targetLocation.distance);
		    if (targetLocation.id = Target::kCenter) {
			// If we're aimed at the center of the target array, just stop here.
			fireControl = kNoTarget;
		    } else {
			// Start the shooter (unless we're aimed at
			// the center of the target array.)
			// TBD: check adjustment range (dsa3) here
			shooter.SetTarget(targetLocation.height,
					  targetLocation.distance,
					  (dsa3-0.5)*2.);
			shooter.Start();
			fireControl = kShooting;
		    }
		} else {
		    printf("Taking another picture...\n");
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
	    printf("Targeting returning to manual control\n");
	    shooter.Stop();
	    if (controlMode == kSpeed)
		EnableSpeedControl();
	    else
		EnableVoltageControl();
	    fireControl = kManual;
	}

	illuminator.Set( dsd8 ? Relay::kOn : Relay::kOff );

	float s = 0.400 + (dsa3 * 0.500);
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

		bool turnLeft10 = joy_right.GetRawButton(3);
		bool turnRight10 = joy_right.GetRawButton(4);
		bool turnLeft3 = joy_right.GetRawButton(5);
		bool turnRight3 = joy_right.GetRawButton(6);
		
		if (turnLeft10 || turnRight10 || turnLeft3 || turnRight3)
		{
		    float angle = turnLeft10 ? -10.
				: turnRight10 ? 10.
				: turnLeft3  ? -3.
				: turnRight3 ? 3.
				: 0.;

		    const float tolerance = 0.50;

		    if (!turnComplete) {
			if (!turning) {
			    EnablePositionControl();
			    turning = true;
			    turnComplete = false;
			}
			if (TurnToPosition(angle, tolerance)) {
			    printf("turn complete: %g %g %g %g\n",
			      GetJaguarPosition(motor_left_1,"motor_left_1"),
			      GetJaguarPosition(motor_left_2,"motor_left_2"),
			      GetJaguarPosition(motor_right_1,"motor_right_1"),
			      GetJaguarPosition(motor_right_2,"motor_right_2"));
			    turnComplete = true;
			}
		    }
		} else {
		    if (turning) {
			DisableMotors();
			if (controlMode == kSpeed)
			    EnableSpeedControl();
			else
			    EnableVoltageControl();
			turning = false;
			turnComplete = false;
		    }

		    switch (driveMode) {
		    case kFlightStick:
			    drive.ArcadeDrive( rightY, -rightT, true );
			    break;
		    case kArcade:
			    drive.ArcadeDrive( rightY, -rightX, true );
			    break;
		    case kXY:
			    if (rightY > 0.10) {
				drive.ArcadeDrive( rightY, rightX, true );
			    } else {
				drive.ArcadeDrive( rightY, -rightX, true );
			    }
			    break;
		    case kTwoStick:
			    drive.TankDrive( rightY, leftY );
			    break;
		    default:
			    printf("ERROR: Invalid drive mode (can't happen)\n");
			    DisableMotors();
			    break;
		    }
		}
#if 0 // disable auto-balance code
	    }
	}
#endif
	balance.Run();
    }
}

void MyRobot::TeleopContinuous()
{
}
