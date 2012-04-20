// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include <math.h>
#include "ShootCommand.h"
#include "MyRobot.h"
#include "Target.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float ShootCommand::cameraWarmup = 1.50;
const float ShootCommand::turnTimeout  = 1.00;
const float ShootCommand::aimTolerance = 0.50;

const char *ShootCommand::StateName( FireControl state )
{
    const char *stateName;

    switch( state ) {
    case kLights:
	stateName = "lights";
	break;
    case kCamera:
	stateName = "camera";
	break;
    case kAction:
	stateName = "action";
	break;
    case kSpinUp:
	stateName = "spin up";
	break;
    case kShoot1:
	stateName = "shoot 1";
	break;
    case kShoot2:
	stateName = "shoot 2";
	break;
    case kShoot3:
	stateName = "shoot 3";
	break;
    case kDone:
    default:
	stateName = "done";
	break;
    }

    return stateName;
}

ShootCommand::ShootCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
    m_timer.Start();
}

void ShootCommand::Start( Target::TargetID targetID, FireControl lastState )
{
    printf("Starting targeting sequence: %s target, end at %s\n",
	Target::TargetName(targetID), StateName(lastState));

    m_targetID = targetID;
    m_lastState = lastState;
    m_fireControl = kDone;

    EnterState(kLights);
}

void ShootCommand::Stop()
{
    OI& oi = m_robot.GetOI();

    // disable drive motors
    m_robot.DisableMotors();
    // return illuminator to manual control
    m_robot.illuminator.Set( oi.Illuminator() ? Relay::kOn : Relay::kOff );
    // but leave the shooter running...
}

void ShootCommand::EnterState( FireControl newState )
{
    OI& oi = m_robot.GetOI();

    printf("EnterState: newState %s\n", StateName(newState));

    // short circuit state progression when we try to leave "lastState"
    if (m_fireControl == m_lastState) {
	newState = kDone;
    }

    switch (newState) {
    case kLights:
	m_robot.illuminator.Set( Relay::kOn );
	break;

    case kCamera:
	m_robot.target.StartAcquisition();
	break;

    case kAction:
	m_robot.EnablePositionControl();
	break;

    case kSpinUp:
	// return illuminator to manual control
	m_robot.illuminator.Set( oi.Illuminator() ? Relay::kOn : Relay::kOff );
	m_robot.shooter.SetTarget( m_targetLocation.height, m_targetLocation.distance );
	m_robot.shooter.Start();
	break;

    case kShoot1:
    case kShoot2:
    case kShoot3:
	m_robot.shooter.Shoot();
	break;

    case kDone:
	break;

    }
    MyRobot::ShowState("Shoot", StateName(newState));
    m_fireControl = newState;
    m_timer.Reset();
}

bool ShootCommand::Run()
{
    OI& oi = m_robot.GetOI();

    switch (m_fireControl) {
    case kLights:
	// must wait a while for the camera to adjust to the lighting change
	if (m_timer.Get() > cameraWarmup) {
	    EnterState(kCamera);
	}
	break;

    case kCamera:
	if (m_robot.target.ProcessingComplete()) {
	    bool targetsFound = m_robot.target.TargetsFound();
	    if (oi.Teach()) {
		printf("Target image processing complete\n");
	    }
	    if (targetsFound) {
		m_targetLocation = m_robot.target.GetTargetLocation(m_targetID);
		if (oi.Teach()) {
		    printf("Target \"%s\" center visible %d, angle %g, distance %g\n",
			Target::TargetName(m_targetLocation.id),
			m_targetLocation.valid,
			m_targetLocation.angle,
			m_targetLocation.distance);
		}
		if (m_targetLocation.valid && (fabs(m_targetLocation.angle) < aimTolerance)) {
		    EnterState(kSpinUp);
		} else {
		    EnterState(kAction);
		}
	    } else {
		if (oi.Teach()) {
		    printf("No targets visible\n");
		}
		// Stop here.  Avoid using EnterState because we want to leave "no targets"
		// on the driver station display as opposed to the default "done".
		m_fireControl = kDone;
		MyRobot::ShowState("Shoot", "no targets");
	    }
	}
	break;

    case kAction:
	bool turnComplete = m_robot.TurnToAngle(m_targetLocation.angle);
	if (turnComplete || (m_timer.Get() > turnTimeout)) {
	    if (oi.Teach()) {
		printf("Auto turn %s: m_angle %g left %g right %g\n",
		   turnComplete ? "complete" : "TIMEOUT", m_targetLocation.angle,
		   m_robot.GetJaguarAngle(m_robot.motor_left,"left"),
		   m_robot.GetJaguarAngle(m_robot.motor_right,"right"));
	    }
	    m_robot.DisableMotors();
	    // Take another picture and check position.
	    EnterState(kCamera);
	}
	break;

    case kSpinUp:
	m_robot.shooter.Run();
	if (m_robot.shooter.IsReady()) {
	    EnterState(kShoot1);
	}
	break;

    case kShoot1:
	m_robot.shooter.Run();
	if (m_robot.shooter.IsReady()) {
	    EnterState(kShoot2);
	}
	break;

    case kShoot2:
	m_robot.shooter.Run();
	if (m_robot.shooter.IsReady()) {
	    EnterState(kShoot3);
	}
	break;

    case kShoot3:
	m_robot.shooter.Run();
	if (m_robot.shooter.IsReady()) {
	    EnterState(kDone);
	}
	break;

    case kDone:
	break;  // stay in this state
    }

    return (m_fireControl == kDone);
}
