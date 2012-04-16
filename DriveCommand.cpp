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
}

void DriveCommand::Start()
{
    m_controlMode = kVoltage;
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

    ControlMode newMode = oi.Brake() ? kSpeed : kVoltage;
    if (newMode != m_controlMode) {
	switch (newMode) {
	case kSpeed:
	    printf("changing control mode to kSpeed\n");
	    m_robot.EnableSpeedControl();
	    break;
	case kVoltage:
	    printf("changing control mode to kVoltage\n");
	    m_robot.EnableVoltageControl();
	    break;
	}
	m_controlMode = newMode;
    }

    float rightY = oi.GetRightY();
    float rightT = oi.GetRightTwist();

    m_robot.drive.ArcadeDrive( rightY, -rightT / 3.0, false );

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

    oi.OnRampLED(m_robot.onRamp.Get());

    switch (oi.Tipper()) {
    case 2:
	m_robot.tipper.Set( false );
	break;
    case 1:
	break;
    case 0:
	m_robot.tipper.Set( true );
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
	if (m_robot.target.ProcessingComplete()) {
	    bool targetsFound = m_robot.target.TargetsFound();
	    if (targetsFound) {
		Target::TargetLocation targetLocation;
		switch (height) {
		case 0:
		    targetLocation = m_robot.target.GetTargetLocation(Target::kBottom);
		    break;
		case 1:
		    Target::TargetLocation targetLeft =
			m_robot.target.GetTargetLocation(Target::kLeft);
		    Target::TargetLocation targetRight =
			m_robot.target.GetTargetLocation(Target::kRight);

		    if (targetLeft.valid) {
			if (targetRight.valid) {
			    // if both are in view, pick the target that's closest to center
			    targetLocation = (fabs(targetLeft.angle) < fabs(targetRight.angle)) 
					    ? targetLeft : targetRight;
			} else {
			    // pick the valid target
			    targetLocation = targetLeft;
			}
		    } else {
			// either left is not valid and right is: pick right
			// or neither one is valid: pick either one
			targetLocation = targetRight;
		    }
		    break;

		case 2:
		    targetLocation = m_robot.target.GetTargetLocation(Target::kTop);
		    break;
		}
		SmartDashboard::Log(
		    (targetLocation.id == Target::kTop) ? "top" :
		    (targetLocation.id == Target::kLeft) ? "left" :
		    (targetLocation.id == Target::kRight) ? "right" : "bottom", "target");
		SmartDashboard::Log(targetLocation.valid,    "target.valid");
		SmartDashboard::Log(targetLocation.distance, "target.distance");
		SmartDashboard::Log(targetLocation.angle,    "target.angle");
		if (targetLocation.valid) {
		    float speed = Shooter::Ballistics(targetLocation.height, targetLocation.distance);
		    SmartDashboard::Log(speed, "target.speed");
		}
	    } else {
		SmartDashboard::Log("",    "target");
		SmartDashboard::Log(false, "target.valid");
		SmartDashboard::Log("",    "target.distance");
		SmartDashboard::Log("",    "target.angle");
		SmartDashboard::Log("",    "target.speed");
	    }
	    m_robot.target.StartAcquisition();
	}

	float speed = 0.300 + (oi.Adjust() * 0.650);
	m_robot.shooter.SetSpeed(speed);
	oi.ReadyLED(m_robot.shooter.IsReady());

	// This will repeat fire if the button is held down.
	if (oi.Shoot()) {
	    m_robot.shooter.Shoot();
	}
    } else {
	oi.ReadyLED(false);
    }
    m_robot.shooter.Run();

    ////////////////////////////////////////

    return false;
}
