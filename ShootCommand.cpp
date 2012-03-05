// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "ShootCommand.h"
#include "MyRobot.h"
#include "Target.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float ShootCommand::turnTolerance = 0.50;

ShootCommand::ShootCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
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
    m_robot.target.StartAcquisition();
    fireControl = kLooking;
    MyRobot::ShowState("Teleop", "Looking");
}

void ShootCommand::Stop()
{
    m_robot.DisableMotors();
    turnTimer.Stop();
}

bool ShootCommand::Run()
{
    OI& oi = m_robot.GetOI();

    switch (fireControl) {
    case kLooking:
	if (m_robot.target.ProcessingComplete()) {
	    printf("Target image processing complete\n");
	    if (m_robot.target.TargetsFound()) {
		printf("Targets found\n");
		Target::TargetID id = oi.TargetTop() ? Target::kTop
				    : oi.TargetLeft() ? Target::kLeft
				    : oi.TargetRight() ? Target::kRight
				    : oi.TargetBottom() ? Target::kBottom
				    : Target::kCenter;  // can't happen

		targetLocation = m_robot.target.GetTargetLocation(id);

		// start turn toward the target
		printf("Starting turn: %g degrees\n", targetLocation.angle);
		m_robot.EnablePositionControl();
		turnTimer.Start();
		fireControl = kTurning;
		MyRobot::ShowState("Teleop", "Turning");
	    } else {
		printf("No targets visible\n");
		fireControl = kNoTarget;
		MyRobot::ShowState("Teleop", "No Targets");
	    }
	}
	break;

    case kTurning:
	bool turnComplete = m_robot.TurnToPosition(targetLocation.angle, turnTolerance);
	if (turnComplete || turnTimer.HasPeriodPassed(2.0)) {
	    printf("Target turn %s: %g %g %g %g\n",
	      turnComplete ? "complete" : "TIMEOUT",
	      m_robot.GetJaguarAngle(m_robot.motor_left_1,"left_1"),
	      m_robot.GetJaguarAngle(m_robot.motor_left_2,"left_2"),
	      m_robot.GetJaguarAngle(m_robot.motor_right_1,"right_1"),
	      m_robot.GetJaguarAngle(m_robot.motor_right_2,"right_2"));

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
		    MyRobot::ShowState("Teleop", "Aimed At Center");
		} else {
		    // Start the shooter.
		    // TBD: change adjustment scaling?
		    printf("Starting shooter, distance = %g\n", targetLocation.distance);
		    m_robot.shooter.SetTarget(targetLocation.height, targetLocation.distance,
					      (oi.Adjust()-0.5)*2.);
		    m_robot.shooter.Start();
		    fireControl = kShooting;
		    MyRobot::ShowState("Teleop", "Shooting");
		}
	    } else {
		printf("Taking another picture...\n");
		// More of the target should be in view now.
		// Take another picture and reposition.
		m_robot.target.StartAcquisition();
		fireControl = kLooking;
		MyRobot::ShowState("Teleop", "Looking (again)");
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

    return false;
}
