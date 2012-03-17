// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include <math.h>
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
}

void DriveCommand::Start()
{
    driveMode = (DriveType) (int) driveChooser.GetSelected();

    controlMode = selectedMode = (ControlMode) (int) controlChooser.GetSelected();
    if (controlMode == kSpeed)
	m_robot.EnableSpeedControl();
    else
	m_robot.EnableVoltageControl();

    MyRobot::ShowState("Drive", "Start");
}

void DriveCommand::Stop()
{
    m_robot.DisableMotors();
}

bool DriveCommand::Run()
{
    OI& oi = m_robot.GetOI();

    ////////////////////////////////////////

    ControlMode newMode = oi.Brake() ? kPosition : selectedMode;
    if (newMode != controlMode) {
	switch (newMode) {
	case kSpeed:
	    printf("changing control mode to kSpeed\n");
	    m_robot.EnableSpeedControl();
	    break;
	case kVoltage:
	    printf("changing control mode to kVoltage\n");
	    m_robot.EnableVoltageControl();
	    break;
	case kPosition:
	    printf("changing control mode to kPosition\n");
	    m_robot.EnablePositionControl();
	    break;
	}
	controlMode = newMode;
    }

    float leftY  = oi.GetLeftY();
    float rightY = oi.GetRightY();
    float rightX = oi.GetRightX();
    float rightT = oi.GetRightTwist();

    if (controlMode == kPosition) {
	// single stick, no turns
	(void) m_robot.DriveToPosition( 44. * rightY, 0. );
    } else {
	switch (driveMode) {
	case kFlightStick:
	    m_robot.drive.ArcadeDrive( rightY, -rightT / 2.0, false );
	    break;
	case kArcade:
	    m_robot.drive.ArcadeDrive( rightY, -rightX / 2.0, false );
	    break;
	case kXY:
	    if (rightY > 0.10) {
		m_robot.drive.ArcadeDrive( rightY, rightX / 2.0, false );
	    } else {
		m_robot.drive.ArcadeDrive( rightY, -rightX / 2.0, false );
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
	if (!m_robot.shooter.IsRunning()) {
	    m_robot.target.StartAcquisition();
	    m_robot.shooter.Start();
	}
	break;
    case 1:	// center-off, no change
	break;
    case 0:	// down, stop
	m_robot.shooter.Stop();
	break;
    }
    if (m_robot.shooter.IsRunning()) {
	int height = oi.Extra();
	Target::TargetID id = (height == 2) ? Target::kTop
			    : (height == 1) ? Target::kLeft
			    : Target::kBottom;

	if (m_robot.target.ProcessingComplete()) {
	    bool targetsFound = m_robot.target.TargetsFound();
	    if (targetsFound) {
		Target::TargetLocation targetLocation = m_robot.target.GetTargetLocation(id);
		SmartDashboard::Log(targetLocation.valid,    "target.valid");
		SmartDashboard::Log(targetLocation.distance, "target.distance");
		SmartDashboard::Log(targetLocation.angle,    "target.angle");
	    } else {
		SmartDashboard::Log(false, "target.valid");
	    }
	    m_robot.target.StartAcquisition();
	}

	float speed = 0.300 + (oi.Adjust() * 0.650);
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
