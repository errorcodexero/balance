// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "ShootCommand.h"
#include "MyRobot.h"
#include "Target.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float ShootCommand::turnTolerance = 0.80;

ShootCommand::ShootCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
    turnTimer.Start();
}

void ShootCommand::Start()
{
    OI& oi = m_robot.GetOI();

    const char *target = oi.TargetTop() ? "top"
		       : oi.TargetLeft() ? "left"
		       : oi.TargetRight() ? "right"
		       : oi.TargetBottom() ? "bottom"
		       : "center";  // can't happen

    printf("Starting targeting sequence: %s\n", target);

    m_robot.DisableMotors();
    m_robot.illuminator.Set( Relay::kOn );
    turnTimer.Reset();
    fireControl = kLights;
    MyRobot::ShowState("Shoot", "Lights");
}

void ShootCommand::Stop()
{
    m_robot.DisableMotors();
    m_robot.shooter.Stop();
}

bool ShootCommand::Run()
{
    OI& oi = m_robot.GetOI();
    Target::TargetID id = oi.TargetTop() ? Target::kTop
			: oi.TargetLeft() ? Target::kLeft
			: oi.TargetRight() ? Target::kRight
			: oi.TargetBottom() ? Target::kBottom
			: Target::kCenter;  // can't happen

    switch (fireControl) {
    case kLights:
	if (turnTimer.Get() > 0.5) {
	    m_robot.target.StartAcquisition();
	    fireControl = kCamera;
	}
	break;
    case kCamera:
	if (m_robot.target.ProcessingComplete()) {
	    printf("Target image processing complete\n");
	    bool targetsFound = m_robot.target.TargetsFound();
	    if (targetsFound) {
		printf("Targets found\n");
		targetLocation = m_robot.target.GetTargetLocation(id);
	    }
	    if (targetsFound) {
		// start turn toward the target
		printf("Starting turn: %g degrees\n", targetLocation.angle);
		m_robot.EnablePositionControl();
		turnTimer.Reset();
		fireControl = kAction;
		MyRobot::ShowState("Shoot", "Turning");
	    } else {
		printf("No targets visible\n");
		fireControl = kNoTarget;
		MyRobot::ShowState("Shoot", "No Targets");
	    }
	}
	break;

    case kAction:
	bool turnComplete = m_robot.TurnToAngle(targetLocation.angle, turnTolerance);
	if (turnComplete || turnTimer.Get() > 2.0) {
	    printf("Target turn %s: %g %g %g\n",
	      turnComplete ? "complete" : "TIMEOUT", targetLocation.angle,
	      m_robot.GetJaguarAngle(m_robot.motor_left,"left"),
	      m_robot.GetJaguarAngle(m_robot.motor_right,"right"));
	    m_robot.DisableMotors();

	    // If the entire target was already visible and we've
	    // completed a turn to that location, assume we've
	    // turned the right amount and fire up the shooter.
	    // Else take another picture.
	    if (targetLocation.valid && turnComplete) {
		m_robot.illuminator.Set( Relay::kOff );  // no more pictures
		if (targetLocation.id = Target::kCenter) {
		    // If we're aimed at the center of the target array, just stop here.
		    printf("Aiming complete, distance = %g\n", targetLocation.distance);
		    fireControl = kNoTarget;
		    MyRobot::ShowState("Shoot", "Aimed At Center");
		} else {
		    // Start the shooter.
		    // TBD: change adjustment scaling?
		    printf("Starting shooter, distance = %g\n", targetLocation.distance);
		    m_robot.shooter.SetTarget(targetLocation.height, targetLocation.distance,
					      (oi.Adjust()-0.5)*2.);
		    m_robot.shooter.Start();
		    fireControl = kShooting;
		    MyRobot::ShowState("Shoot", "Shooting");
		}
	    } else {
		printf("Taking another picture...\n");
		// More of the target should be in view now.
		// Take another picture and reposition.
		m_robot.target.StartAcquisition();
		fireControl = kCamera;
		MyRobot::ShowState("Shoot", "Looking (again)");
	    }
	}
	break;

    case kShooting:
	// Here's where a "ball ready" sensor would be helpful.
	if (m_robot.shooter.IsReady()) {
	    m_robot.shooter.Shoot();
	}
	m_robot.shooter.Run();
	break;

    case kNoTarget:
	break;	// stay in this state until gunner releases button
    }

    return true;	// allow interruption at any point
}
