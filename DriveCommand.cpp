// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "DriveCommand.h"
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

DriveCommand::DriveCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
    driveChooser.AddDefault("FlightStick", (void *) kFlightStick);
    driveChooser.AddObject("Arcade",       (void *) kArcade);
    driveChooser.AddObject("X-Y",          (void *) kXY);
    driveChooser.AddObject("TwoStick",     (void *) kTwoStick);
    SmartDashboard::GetInstance()->PutData("Drive", &driveChooser);

    controlChooser.AddDefault("Voltage", (void *) kVoltage);
    controlChooser.AddObject("Speed",    (void *) kSpeed);
    SmartDashboard::GetInstance()->PutData("Control", &controlChooser);

//  MyRobot::ShowState("Initialize", "Manual Drive");
}

void DriveCommand::Start()
{
    driveMode = (DriveType) (int) driveChooser.GetSelected();

    controlMode = (ControlMode) (int) controlChooser.GetSelected();
    if (controlMode == kSpeed)
	m_robot.EnableSpeedControl();
    else
	m_robot.EnableVoltageControl();

    MyRobot::ShowState("Teleop", "Drive");
}

void DriveCommand::Stop()
{
    m_robot.DisableMotors();
}

bool DriveCommand::Run()
{
    OI& oi = m_robot.GetOI();

    ////////////////////////////////////////

    float leftY  = oi.GetLeftY();
    float rightY = oi.GetRightY();
    float rightX = oi.GetRightX();
    float rightT = oi.GetRightTwist();

    switch (driveMode) {
    case kFlightStick:
	m_robot.drive.ArcadeDrive( rightY, -rightT, false );
	break;
    case kArcade:
	m_robot.drive.ArcadeDrive( rightY, -rightX, false );
	break;
    case kXY:
	if (rightY > 0.10) {
	    m_robot.drive.ArcadeDrive( rightY, rightX, false );
	} else {
	    m_robot.drive.ArcadeDrive( rightY, -rightX, false );
	}
	break;
    case kTwoStick:
	m_robot.drive.TankDrive( rightY, leftY );
	break;
    default:
	printf("ERROR: Invalid drive mode (can't happen)\n");
	m_robot.DisableMotors();
	break;
    }

    ////////////////////////////////////////

    switch (oi.BallPickup()) {
    case 2:	// up, forward
	m_robot.pickup.Forward();
	break;
    case 1:	// center, off
        m_robot.pickup.Stop();
	break;
    case 0:	// down, reverse
	m_robot.pickup.Reverse();
	break;
    }

    ////////////////////////////////////////

    bool trigger = oi.GetRightTrigger();
    if (trigger != m_prevTrigger) {
	m_robot.cowcatcher.Set(trigger);
    }
    m_prevTrigger = trigger;

    switch (oi.Cowcatcher()) {
    case 2:
	m_robot.cowcatcher.Set( false );
	break;
    case 1:
	break;
    case 0:
	m_robot.cowcatcher.Set( true );
	break;
    }

    ////////////////////////////////////////

    m_robot.illuminator.Set( oi.Illuminator() ? Relay::kOn : Relay::kOff );

    ////////////////////////////////////////

    switch (oi.Shooter()) {
    case 2:	// up, start
	m_robot.shooter.Start();
	break;
    case 1:	// center-off, no change
	break;
    case 0:	// down, stop
	m_robot.shooter.Stop();
	break;
    }
    if (m_robot.shooter.IsRunning()) {
	float speed = 0.450 + (oi.Adjust() * 0.500);
	m_robot.shooter.SetSpeed(speed);

	// This will repeat fire if the button is held down.
	if (oi.Shoot()) {
	    m_robot.shooter.Shoot();
	}
    }
    m_robot.shooter.Run();

    ////////////////////////////////////////

    return false;
}
