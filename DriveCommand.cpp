// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include <math.h>
#include "DriveCommand.h"
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

#define	DEADZONE 0.035

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

    if (fabs(rightY) < DEADZONE) rightY = 0.;
    if (fabs(rightT) < DEADZONE) rightT = 0.;

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
	    m_robot.shooter.SetManual();
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
