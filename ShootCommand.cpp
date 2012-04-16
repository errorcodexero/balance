// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include <math.h>
#include "ShootCommand.h"
#include "MyRobot.h"
#include "Target.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float ShootCommand::aimTolerance = 0.50;

ShootCommand::ShootCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
    turnTimer.Start();
}

void ShootCommand::Start()
{
    OI& oi = m_robot.GetOI();
    const char *targetName;

    if (oi.TargetTop()) {
	m_targetID = Target::kTop;
	targetName = "top";
    } else if (oi.TargetLeft()) {
	m_targetID = Target::kLeft;
	targetName = "left";
    } else if (oi.TargetRight()) {
	m_targetID = Target::kRight;
	targetName = "right";
    } else if (oi.TargetBottom()) {
	m_targetID = Target::kBottom;
	targetName = "bottom";
    } else {
	// shooter started without a target
	m_targetID = Target::kCenter;
	targetName = "center";
    }

    printf("Starting targeting sequence: %s\n", targetName);

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
    switch (fireControl) {
    case kLights:
	// wait a while for the camera to adjust to the lighting change
	if (turnTimer.Get() > 1.5) {
	    m_robot.target.StartAcquisition();
	    fireControl = kCamera;
	    MyRobot::ShowState("Shoot", "Camera");
	}
	break;
    case kCamera:
	if (m_robot.target.ProcessingComplete()) {
	    printf("Target image processing complete\n");
	    bool targetsFound = m_robot.target.TargetsFound();
	    if (targetsFound) {
		printf("Targets found\n");
		targetLocation = m_robot.target.GetTargetLocation(m_targetID);
		// are we already on target?
		OI& oi = m_robot.GetOI();
		if (oi.Extra() == 0) {
		    // camera only - no motion
		    printf("Targeting location: visible %d, angle %g, distance %g\n",
			targetLocation.valid, targetLocation.angle, targetLocation.distance);
		    m_robot.illuminator.Set( Relay::kOff );  // no more pictures
		    fireControl = kNoTarget;
		    MyRobot::ShowState("Shoot", "Complete");
		} else if (targetLocation.valid && (fabs(targetLocation.angle) < aimTolerance)) {
		    printf("Aiming complete: angle %g, distance %g\n",
			targetLocation.angle, targetLocation.distance);
		    m_robot.illuminator.Set( Relay::kOff );  // no more pictures
		    if (oi.Extra() == 1 || targetLocation.id == Target::kCenter) {
			// If we're aimed at the center of the target array, just stop here.
			fireControl = kNoTarget;
			MyRobot::ShowState("Shoot", "Complete");
		    } else {
			// Start the shooter.
			// TBD: change adjustment scaling?
			printf("Starting shooter...\n");
			m_robot.shooter.SetTarget(targetLocation.height,
				targetLocation.distance, (oi.Adjust()-0.5)*2.);
			m_robot.shooter.Start();
			fireControl = kShooting;
			MyRobot::ShowState("Shoot", "Shooting");
		    }
		} else {
		    printf("Starting turn: %g degrees\n", targetLocation.angle);
		    m_robot.EnablePositionControl();
		    turnTimer.Reset();
		    fireControl = kAction;
		    MyRobot::ShowState("Shoot", "Action");
		}
	    } else {
		printf("No targets visible\n");
		fireControl = kNoTarget;
		MyRobot::ShowState("Shoot", "No Targets");
	    }
	}
	break;

    case kAction:
	bool turnComplete = m_robot.TurnToAngle(targetLocation.angle);
	if (turnComplete || turnTimer.Get() > 2.0) {
	    printf("Target turn %s: %g %g %g\n",
	      turnComplete ? "complete" : "TIMEOUT", targetLocation.angle,
	      m_robot.GetJaguarAngle(m_robot.motor_left,"left"),
	      m_robot.GetJaguarAngle(m_robot.motor_right,"right"));
	    m_robot.DisableMotors();

	    printf("Taking another picture...\n");
	    // More of the target should be in view now.
	    // Take another picture and reposition.
	    m_robot.target.StartAcquisition();
	    fireControl = kCamera;
	    MyRobot::ShowState("Shoot", "Camera");
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
