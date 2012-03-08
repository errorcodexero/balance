// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "AutoCommand.h"
#include "MyRobot.h"
#include "Target.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float AutoCommand::shotTimeLimit = 4.0;
const float AutoCommand::turnTimeLimit = 2.0;
const float AutoCommand::driveTimeLimit = 4.0;
const float AutoCommand::turnTolerance = 0.5;
const float AutoCommand::driveTolerance = 2.0;

AutoCommand::AutoCommand( MyRobot& theRobot ) : m_robot(theRobot), selector(3)
{
    autoState = kStopped;
    autoTimer.Start();
}

void AutoCommand::Start()
{
    // get autonomous program from analog channel
    autoSequence = (int)(selector.GetVoltage() * 9.0 / 5.0 + 0.5);
    printf("Starting auto sequence %d\n", autoSequence);

    switch (autoSequence) {
    case 0: // no autonomous
	printf("AutoCommand: autonomous mode disabled\n");
	autoState = kStopped;
	return;

    case 1: // left side, no bridge
	shotDistance = 115.5;
	turnAngle = -21.4;
	driveDistance = 0.0;
	break;

    case 2: // center, no bridge
	shotDistance = 109.0;
	turnAngle = 0.0;
	driveDistance = 0.0;
	break;

    case 3: // right side, no bridge
	shotDistance = 115.5;
	turnAngle = 21.4;
	driveDistance = 0.0;
	break;

    case 4: // left side, center bridge
	shotDistance = 115.5;
	turnAngle = -(21.4 + 24.8);
	driveDistance = 150.;
	break;

    case 5: // center, center bridge
	shotDistance = 109.0;
	turnAngle = 0.0;
	driveDistance = 136.;
	break;

    case 6: // right side, center bridge
	shotDistance = 115.5;
	turnAngle = 21.4 + 24.8;
	driveDistance = 150.;
	break;

    case 7: // left side, right bridge
	shotDistance = 115.5;
	// either 55.5 is wrong or large turns need recalibration
	// turnAngle = -(21.4 + 55.5);
	turnAngle = -56.;
	driveDistance = 241.;
	break;

    case 8: // center, right bridge
	shotDistance = 109.0;
	turnAngle = -44.9;
	driveDistance = 193.;
	break;

    case 9: // right side, right bridge
	shotDistance = 115.5;
	turnAngle = 21.4 - 28.2;
	driveDistance = 155.;
	break;

    default: // can't happen
	printf("AutoCommand: invalid sequence number\n");
	autoState = kStopped;
	return;
    }

    m_robot.DisableMotors();
    m_robot.shooter.InitShooter();
    printf("Starting shooter, distance = %g\n", shotDistance);
    m_robot.shooter.SetTarget(2, shotDistance, 0.);
    m_robot.shooter.Start();
    MyRobot::ShowState("Autonomous", "Shoot");
    shotCount = 0;
    autoState = kShooting;
    autoTimer.Reset();
}

void AutoCommand::Stop()
{
    m_robot.DisableMotors();
    m_robot.shooter.Stop();
    autoState = kStopped;
}

bool AutoCommand::Run()
{
    m_robot.shooter.Run();

    switch (autoState) {
	case kShooting: {
	    bool shooterReady = m_robot.shooter.IsReady();

	    if (!shooterReady && (autoTimer.Get() < shotTimeLimit))
		break;	// just wait

	    if (shooterReady) {
		printf("Shoot!\n");
		if (++shotCount <= 2) {
		    m_robot.shooter.Shoot();
		    printf("waiting for next shot...\n");
		    autoTimer.Reset();
		    break;  // stay in this state
		}
		printf("Done shooting\n");
	    } else {
		printf("Shooter TIMEOUT\n");
	    }

	    m_robot.shooter.Stop();
	    if (autoSequence < 4) {
		autoState = kStopped;
	    } else {
		printf("Preparing to move...\n");
		m_robot.EnablePositionControl();
		autoState = kTurning;
		autoTimer.Reset();
	    }
	    break;
	}

	case kTurning: {
	    bool turnComplete = m_robot.TurnToAngle(turnAngle, turnTolerance);

	    if (!turnComplete && (autoTimer.Get() < turnTimeLimit))
		break;	// just wait

	    if (turnComplete) {
		m_robot.cowcatcher.Set(true);
		autoState = kDriving;
		autoTimer.Reset();
	    } else {
		// turn failed, so it isn't safe to drive
		m_robot.DisableMotors();
		autoState = kStopped;
	    }
	    break;
	}

	case kDriving: {
	    bool driveComplete = m_robot.DriveToPosition(driveDistance, driveTolerance);

	    if (!driveComplete && (autoTimer.Get() < driveTimeLimit))
		break;

	    // we're done
	    m_robot.DisableMotors();
	    autoState = kStopped;
	    break;
	}

	case kStopped:
	    return true;
    }

    return false;
}

